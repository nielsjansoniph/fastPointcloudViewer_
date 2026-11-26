#include "CloudLoader.h"

#include <algorithm>
#include <cctype>
#include <charconv>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <future>
#include <random>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <type_traits>

#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>

namespace {

constexpr std::size_t kMaxChunkPoints = 65536;   // octree leaves are split above this size
constexpr int kMaxOctreeDepth = 24;              // stops splitting (many identical points)
constexpr int kParallelDepth = 2;                // octree levels that spawn threads (8^2 tasks)

void checkCancel(const LoadProgress& progress)
{
    if (progress.cancel.load(std::memory_order_relaxed))
        throw LoadCancelled();
}

// Appends a point and grows the bounding box; skips invalid points
inline void addPoint(CloudData& d, const glm::vec3& p)
{
    d.positions.push_back({p});
    d.min = glm::min(d.min, p);
    d.max = glm::max(d.max, p);
}

// ---------------------------------------------------------------- PCL (fallback, no progress)

template <typename PointT>
void extract(const pcl::PointCloud<PointT>& cloud, CloudData& d)
{
    d.positions.reserve(cloud.points.size());
    constexpr bool kColor = std::is_same_v<PointT, pcl::PointXYZRGBA>;
    if constexpr (kColor) d.colors.reserve(cloud.points.size());
    for (const auto& p : cloud.points){
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
            continue;
        addPoint(d, glm::vec3(p.x, p.y, p.z));
        if constexpr (kColor)
            d.colors.push_back({p.r, p.g, p.b, 255});
    }
}

void readWithPcl(const std::string& filename, const std::string& ext, CloudData& d, LoadProgress& progress)
{
    progress.fraction = -1.0f;
    progress.stage = "Reading (no progress available for this format)";
    // Read the file once into a generic cloud, then convert depending on the fields
    pcl::PCLPointCloud2 blob;
    int result = -1;
    if (ext == ".ply"){
        pcl::PLYReader reader;
        result = reader.read(filename, blob);
    }
    else {
        pcl::PCDReader reader;
        result = reader.read(filename, blob);
    }
    if (result < 0)
        throw std::runtime_error("could not read " + filename);
    checkCancel(progress);

    bool hasColor = false;
    for (const auto& field : blob.fields)
        if (field.name == "rgb" || field.name == "rgba")
            hasColor = true;
    progress.stage = "Converting";
    if (hasColor){
        pcl::PointCloud<pcl::PointXYZRGBA> cloud;
        pcl::fromPCLPointCloud2(blob, cloud);
        blob = pcl::PCLPointCloud2();   // free the raw data early
        extract(cloud, d);
    }
    else {
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::fromPCLPointCloud2(blob, cloud);
        blob = pcl::PCLPointCloud2();
        extract(cloud, d);
    }
}

// ---------------------------------------------------------------- binary PLY (own reader)

enum class PlyType { Int8, UInt8, Int16, UInt16, Int32, UInt32, Float32, Float64, Invalid };

PlyType parsePlyType(const std::string& s)
{
    if (s == "char" || s == "int8") return PlyType::Int8;
    if (s == "uchar" || s == "uint8") return PlyType::UInt8;
    if (s == "short" || s == "int16") return PlyType::Int16;
    if (s == "ushort" || s == "uint16") return PlyType::UInt16;
    if (s == "int" || s == "int32") return PlyType::Int32;
    if (s == "uint" || s == "uint32") return PlyType::UInt32;
    if (s == "float" || s == "float32") return PlyType::Float32;
    if (s == "double" || s == "float64") return PlyType::Float64;
    return PlyType::Invalid;
}

std::size_t plyTypeSize(PlyType t)
{
    switch (t){
    case PlyType::Int8: case PlyType::UInt8: return 1;
    case PlyType::Int16: case PlyType::UInt16: return 2;
    case PlyType::Int32: case PlyType::UInt32: case PlyType::Float32: return 4;
    case PlyType::Float64: return 8;
    default: return 0;
    }
}

template <typename T>
inline T load(const char* p) { T v; std::memcpy(&v, p, sizeof(T)); return v; }

inline double plyValue(const char* p, PlyType t)
{
    switch (t){
    case PlyType::Int8: return load<std::int8_t>(p);
    case PlyType::UInt8: return load<std::uint8_t>(p);
    case PlyType::Int16: return load<std::int16_t>(p);
    case PlyType::UInt16: return load<std::uint16_t>(p);
    case PlyType::Int32: return load<std::int32_t>(p);
    case PlyType::UInt32: return load<std::uint32_t>(p);
    case PlyType::Float32: return load<float>(p);
    case PlyType::Float64: return load<double>(p);
    default: return 0.0;
    }
}

// Color component as 0..255 (uchar as is, ushort scaled, float/double 0..1)
inline std::uint8_t plyColor(const char* p, PlyType t)
{
    double v = plyValue(p, t);
    if (t == PlyType::UInt16) v /= 257.0;
    else if (t == PlyType::Float32 || t == PlyType::Float64) v *= 255.0;
    return (std::uint8_t)std::clamp(v + 0.5, 0.0, 255.0);
}

struct PlyProperty { std::string name; PlyType type = PlyType::Invalid; bool isList = false; std::size_t offset = 0; };
struct PlyElement { std::string name; std::size_t count = 0; std::vector<PlyProperty> props; std::size_t stride = 0; };

// Reads binary little endian PLY files with plain vertex properties directly into CloudData,
// with progress. Returns false for anything else (ascii, big endian, packed rgb, lists
// before the vertices), which is then read with PCL.
bool readPlyBinary(const std::string& filename, CloudData& d, LoadProgress& progress)
{
    std::ifstream in(filename, std::ios::binary);
    if (!in)
        throw std::runtime_error("could not read " + filename);

    std::string line;
    if (!std::getline(in, line) || line.rfind("ply", 0) != 0)
        return false;
    bool binaryLE = false;
    std::vector<PlyElement> elements;
    while (std::getline(in, line)){
        if (!line.empty() && line.back() == '\r') line.pop_back();
        std::istringstream ls(line);
        std::string key;
        ls >> key;
        if (key == "format"){
            std::string fmt;
            ls >> fmt;
            binaryLE = (fmt == "binary_little_endian");
        }
        else if (key == "element"){
            PlyElement e;
            ls >> e.name >> e.count;
            elements.push_back(e);
        }
        else if (key == "property" && !elements.empty()){
            PlyProperty p;
            std::string type;
            ls >> type;
            if (type == "list"){
                p.isList = true;
                std::string countType, itemType;
                ls >> countType >> itemType;
            }
            else
                p.type = parsePlyType(type);
            ls >> p.name;
            elements.back().props.push_back(p);
        }
        else if (key == "end_header")
            break;
    }
    if (!binaryLE || !in)
        return false;

    // Skip the elements before "vertex" (only possible without list properties)
    std::uint64_t skip = 0;
    const PlyElement* vertex = nullptr;
    for (auto& e : elements){
        for (auto& p : e.props){
            if (p.isList || p.type == PlyType::Invalid){
                e.stride = 0;
                break;
            }
            p.offset = e.stride;
            e.stride += plyTypeSize(p.type);
        }
        if (e.name == "vertex"){
            vertex = &e;
            break;
        }
        if (e.stride == 0 && e.count > 0)
            return false;
        skip += (std::uint64_t)e.stride * e.count;
    }
    if (!vertex || vertex->stride == 0)
        return false;

    const PlyProperty* xyz[3] = {nullptr, nullptr, nullptr};
    const PlyProperty* rgb[3] = {nullptr, nullptr, nullptr};
    for (const auto& p : vertex->props){
        if (p.name == "x") xyz[0] = &p;
        else if (p.name == "y") xyz[1] = &p;
        else if (p.name == "z") xyz[2] = &p;
        else if (p.name == "red" || p.name == "diffuse_red") rgb[0] = &p;
        else if (p.name == "green" || p.name == "diffuse_green") rgb[1] = &p;
        else if (p.name == "blue" || p.name == "diffuse_blue") rgb[2] = &p;
        else if (p.name == "rgb" || p.name == "rgba") return false;   // packed colors: PCL handles them
    }
    if (!xyz[0] || !xyz[1] || !xyz[2])
        return false;
    bool hasColor = rgb[0] && rgb[1] && rgb[2];
    bool xyzFloat = xyz[0]->type == PlyType::Float32 && xyz[1]->type == PlyType::Float32 && xyz[2]->type == PlyType::Float32;
    if (skip)
        in.seekg((std::streamoff)skip, std::ios::cur);

    const std::size_t count = vertex->count;
    const std::size_t stride = vertex->stride;
    d.positions.reserve(count);
    if (hasColor) d.colors.reserve(count);

    const std::size_t batch = std::max<std::size_t>(1, (16u << 20) / stride);   // ~16 MB per read
    std::vector<char> buffer(batch * stride);
    for (std::size_t done = 0; done < count; ){
        checkCancel(progress);
        std::size_t n = std::min(batch, count - done);
        in.read(buffer.data(), (std::streamsize)(n * stride));
        if ((std::size_t)in.gcount() != n * stride)
            throw std::runtime_error("file is truncated: " + filename);
        for (std::size_t i = 0; i < n; i++){
            const char* rec = buffer.data() + i * stride;
            glm::vec3 p;
            if (xyzFloat)
                p = glm::vec3(load<float>(rec + xyz[0]->offset), load<float>(rec + xyz[1]->offset), load<float>(rec + xyz[2]->offset));
            else
                p = glm::vec3((float)plyValue(rec + xyz[0]->offset, xyz[0]->type),
                              (float)plyValue(rec + xyz[1]->offset, xyz[1]->type),
                              (float)plyValue(rec + xyz[2]->offset, xyz[2]->type));
            if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
                continue;
            addPoint(d, p);
            if (hasColor)
                d.colors.push_back({plyColor(rec + rgb[0]->offset, rgb[0]->type),
                                    plyColor(rec + rgb[1]->offset, rgb[1]->type),
                                    plyColor(rec + rgb[2]->offset, rgb[2]->type), 255});
        }
        done += n;
        progress.fraction = (float)done / (float)count;
    }
    return true;
}

// ---------------------------------------------------------------- PCD (own reader)

struct PcdField { std::string name; std::size_t size = 4; char type = 'F'; std::size_t count = 1; std::size_t offset = 0; std::size_t index = 0; };

// LZF decompression (format of liblzf, used by PCL for binary_compressed). Returns the
// number of bytes written, 0 on corrupt data.
std::size_t lzfDecompress(const std::uint8_t* in, std::size_t inLen, std::uint8_t* out, std::size_t outLen)
{
    const std::uint8_t* ip = in;
    const std::uint8_t* inEnd = in + inLen;
    std::uint8_t* op = out;
    std::uint8_t* outEnd = out + outLen;
    while (ip < inEnd){
        unsigned ctrl = *ip++;
        if (ctrl < 32){                              // literal run of ctrl + 1 bytes
            ctrl++;
            if (op + ctrl > outEnd || ip + ctrl > inEnd) return 0;
            std::memcpy(op, ip, ctrl);
            op += ctrl;
            ip += ctrl;
        }
        else {                                       // back reference
            std::size_t len = ctrl >> 5;
            std::size_t back = ((ctrl & 0x1f) << 8) + 1;
            if (ip >= inEnd) return 0;
            if (len == 7){
                len += *ip++;
                if (ip >= inEnd) return 0;
            }
            back += *ip++;
            len += 2;
            if (op + len > outEnd || back > (std::size_t)(op - out)) return 0;
            const std::uint8_t* ref = op - back;
            for (; len; --len) *op++ = *ref++;       // may overlap: byte by byte
        }
    }
    return (std::size_t)(op - out);
}

// Packed PCL color (rgb as float bits or rgba as uint32): 0xAARRGGBB
inline Rgba8 unpackPclColor(std::uint32_t v)
{
    return {(std::uint8_t)((v >> 16) & 0xFF), (std::uint8_t)((v >> 8) & 0xFF), (std::uint8_t)(v & 0xFF), 255};
}

inline bool pcdCoordinate(const char* p, const PcdField& f, float& out)
{
    if (f.size == 4) out = load<float>(p);
    else out = (float)load<double>(p);
    return true;
}

// Reads PCD files (DATA ascii, binary or binary_compressed) with x/y/z as float or double and
// optionally a packed rgb/rgba field, with progress. Returns false for anything else (PCL).
bool readPcd(const std::string& filename, CloudData& d, LoadProgress& progress)
{
    std::ifstream in(filename, std::ios::binary);
    if (!in)
        throw std::runtime_error("could not read " + filename);

    std::vector<PcdField> fields;
    std::size_t width = 0, height = 1, points = 0;
    std::string data, line;
    auto words = [](std::istringstream& ls){ std::vector<std::string> w; std::string s; while (ls >> s) w.push_back(s); return w; };
    while (std::getline(in, line)){
        if (!line.empty() && line.back() == '\r') line.pop_back();
        if (line.empty() || line[0] == '#') continue;
        std::istringstream ls(line);
        std::string key;
        ls >> key;
        std::vector<std::string> w = words(ls);
        if (key == "FIELDS" || key == "COLUMNS"){
            fields.assign(w.size(), PcdField());
            for (std::size_t i = 0; i < w.size(); i++) fields[i].name = w[i];
        }
        else if (key == "SIZE" || key == "TYPE" || key == "COUNT"){
            if (w.size() != fields.size()) return false;
            for (std::size_t i = 0; i < w.size(); i++){
                if (key == "SIZE") fields[i].size = (std::size_t)std::stoul(w[i]);
                else if (key == "TYPE") fields[i].type = w[i].empty() ? '?' : w[i][0];
                else fields[i].count = (std::size_t)std::stoul(w[i]);
            }
        }
        else if (key == "WIDTH" && !w.empty()) width = (std::size_t)std::stoull(w[0]);
        else if (key == "HEIGHT" && !w.empty()) height = (std::size_t)std::stoull(w[0]);
        else if (key == "POINTS" && !w.empty()) points = (std::size_t)std::stoull(w[0]);
        else if (key == "DATA"){
            data = w.empty() ? "" : w[0];
            break;
        }
    }
    if (!in || fields.empty())
        return false;
    if (points == 0) points = width * height;

    // layout of one point (binary / ascii) and the fields we need
    std::size_t stride = 0, index = 0;
    const PcdField* xyz[3] = {nullptr, nullptr, nullptr};
    const PcdField* rgb = nullptr;
    for (auto& f : fields){
        f.offset = stride;
        f.index = index;
        stride += f.size * f.count;
        index += f.count;
    }
    for (const auto& f : fields){
        bool coord = f.type == 'F' && (f.size == 4 || f.size == 8) && f.count == 1;
        if (f.name == "x" && coord) xyz[0] = &f;
        else if (f.name == "y" && coord) xyz[1] = &f;
        else if (f.name == "z" && coord) xyz[2] = &f;
        else if ((f.name == "rgb" || f.name == "rgba") && f.size == 4 && f.count == 1) rgb = &f;
    }
    if (!xyz[0] || !xyz[1] || !xyz[2] || stride == 0)
        return false;
    if (data != "ascii" && data != "binary" && data != "binary_compressed")
        return false;

    d.positions.reserve(points);
    if (rgb) d.colors.reserve(points);
    auto add = [&](const glm::vec3& p, std::uint32_t color){
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
            return;
        addPoint(d, p);
        if (rgb) d.colors.push_back(unpackPclColor(color));
    };

    if (data == "binary"){
        const std::size_t batch = std::max<std::size_t>(1, (16u << 20) / stride);   // ~16 MB per read
        std::vector<char> buffer(batch * stride);
        for (std::size_t done = 0; done < points; ){
            checkCancel(progress);
            std::size_t n = std::min(batch, points - done);
            in.read(buffer.data(), (std::streamsize)(n * stride));
            if ((std::size_t)in.gcount() != n * stride)
                throw std::runtime_error("file is truncated: " + filename);
            for (std::size_t i = 0; i < n; i++){
                const char* rec = buffer.data() + i * stride;
                glm::vec3 p;
                for (int a = 0; a < 3; a++) pcdCoordinate(rec + xyz[a]->offset, *xyz[a], p[a]);
                add(p, rgb ? load<std::uint32_t>(rec + rgb->offset) : 0u);
            }
            done += n;
            progress.fraction = (float)done / (float)points;
        }
    }
    else if (data == "binary_compressed"){
        // [uint32 compressed size][uint32 uncompressed size][LZF data]; decompressed, the
        // fields are stored one after the other (all x, then all y, ...)
        std::uint32_t sizes[2];
        in.read((char*)sizes, sizeof(sizes));
        if (!in) throw std::runtime_error("file is truncated: " + filename);
        std::vector<std::uint8_t> packed(sizes[0]);
        const std::size_t step = 16u << 20;
        for (std::size_t done = 0; done < packed.size(); ){
            checkCancel(progress);
            std::size_t n = std::min(step, packed.size() - done);
            in.read((char*)packed.data() + done, (std::streamsize)n);
            if ((std::size_t)in.gcount() != n) throw std::runtime_error("file is truncated: " + filename);
            done += n;
            progress.fraction = 0.5f * (float)done / (float)packed.size();
        }
        progress.stage = "Decompressing";
        std::vector<std::uint8_t> raw(sizes[1]);
        if (sizes[1] < stride * points || lzfDecompress(packed.data(), packed.size(), raw.data(), raw.size()) != raw.size())
            throw std::runtime_error("corrupt compressed data: " + filename);
        packed = std::vector<std::uint8_t>();
        progress.stage = "Reading";
        auto column = [&](const PcdField* f){ return (const char*)raw.data() + f->offset * points; };
        const char* cx = column(xyz[0]);
        const char* cy = column(xyz[1]);
        const char* cz = column(xyz[2]);
        const char* cc = rgb ? column(rgb) : nullptr;
        for (std::size_t i = 0; i < points; i++){
            if ((i & 0xFFFFF) == 0){
                checkCancel(progress);
                progress.fraction = 0.5f + 0.5f * (float)i / (float)points;
            }
            glm::vec3 p;
            pcdCoordinate(cx + i * xyz[0]->size, *xyz[0], p.x);
            pcdCoordinate(cy + i * xyz[1]->size, *xyz[1], p.y);
            pcdCoordinate(cz + i * xyz[2]->size, *xyz[2], p.z);
            add(p, cc ? load<std::uint32_t>(cc + i * 4) : 0u);
        }
    }
    else {   // ascii: one point per line, the values of all fields separated by spaces
        std::error_code ec;
        double fileSize = (double)std::filesystem::file_size(filename, ec);
        std::size_t wanted[4] = {xyz[0]->index, xyz[1]->index, xyz[2]->index, rgb ? rgb->index : (std::size_t)-1};
        std::size_t lastIndex = std::max({wanted[0], wanted[1], wanted[2], rgb ? wanted[3] : 0});
        for (std::size_t n = 0; n < points && std::getline(in, line); n++){
            if ((n & 0xFFFF) == 0){
                checkCancel(progress);
                if (fileSize > 0) progress.fraction = (float)((double)in.tellg() / fileSize);
            }
            const char* s = line.data();
            const char* e = s + line.size();
            float coord[3] = {NAN, NAN, NAN};
            std::uint32_t color = 0;
            for (std::size_t v = 0; v <= lastIndex && s < e; v++){
                while (s < e && (*s == ' ' || *s == '\t')) ++s;
                const char* tokEnd = s;
                while (tokEnd < e && *tokEnd != ' ' && *tokEnd != '\t' && *tokEnd != '\r') ++tokEnd;
                for (int a = 0; a < 3; a++)
                    if (v == wanted[a]) std::from_chars(s, tokEnd, coord[a]);
                if (rgb && v == wanted[3]){
                    // PCL writes rgb as an integer (current versions) or as the float with
                    // the same bits (older ones): integer if the whole token is one
                    std::uint64_t u = 0;
                    auto [end, err] = std::from_chars(s, tokEnd, u);
                    if (err == std::errc() && end == tokEnd)
                        color = (std::uint32_t)u;
                    else {
                        float f = 0.0f;
                        std::from_chars(s, tokEnd, f);
                        std::memcpy(&color, &f, 4);
                    }
                }
                s = tokEnd;
            }
            add(glm::vec3(coord[0], coord[1], coord[2]), color);
        }
    }
    return true;
}

// ---------------------------------------------------------------- OBJ (vertices only)

// Reads the vertices ("v x y z [r g b]") of a Wavefront OBJ file as points; faces, normals
// and texture coordinates are ignored. Vertex colors are taken if every vertex has one
// (range 0..1, or 0..255 if any component is > 1). Points are rotated from Y-up to Z-up.
void readObj(const std::string& filename, CloudData& d, LoadProgress& progress)
{
    std::ifstream in(filename, std::ios::binary);
    if (!in)
        throw std::runtime_error("could not read " + filename);
    std::error_code ec;
    double fileSize = (double)std::filesystem::file_size(filename, ec);

    std::vector<glm::vec3> rgb;     // float colors, converted once the range is known
    bool allColored = true;
    float maxComponent = 0.0f;

    std::string line;
    for (std::size_t lineNo = 0; std::getline(in, line); lineNo++){
        if ((lineNo & 0xFFFF) == 0){
            checkCancel(progress);
            if (fileSize > 0) progress.fraction = (float)((double)in.tellg() / fileSize);
        }
        if (line.size() < 2 || line[0] != 'v' || (line[1] != ' ' && line[1] != '\t'))
            continue;   // only plain vertices, not vn/vt/f/...
        float val[6];
        int n = 0;
        // from_chars is locale-independent (strtof would expect ',' in a German locale)
        const char* s = line.data() + 2;
        const char* e = line.data() + line.size();
        while (n < 6){
            while (s < e && (*s == ' ' || *s == '\t')) ++s;
            auto [end, err] = std::from_chars(s, e, val[n]);
            if (err != std::errc()) break;
            ++n;
            s = end;
        }
        if (n < 3)
            continue;
        // OBJ is Y-up, the viewer is Z-up: rotate +90° about X, (x, y, z) -> (x, -z, y)
        glm::vec3 pos(val[0], -val[2], val[1]);
        if (!std::isfinite(pos.x) || !std::isfinite(pos.y) || !std::isfinite(pos.z))
            continue;
        addPoint(d, pos);
        if (n == 6 && allColored){
            rgb.emplace_back(val[3], val[4], val[5]);
            maxComponent = std::max({maxComponent, val[3], val[4], val[5]});
        }
        else
            allColored = false;
    }

    if (allColored && !rgb.empty() && rgb.size() == d.positions.size()){
        float scale = maxComponent > 1.0f ? 1.0f : 255.0f;
        d.colors.reserve(rgb.size());
        for (const auto& c : rgb){
            glm::vec3 v = glm::clamp(c * scale, 0.0f, 255.0f) + 0.5f;
            d.colors.push_back({(std::uint8_t)v.r, (std::uint8_t)v.g, (std::uint8_t)v.b, 255});
        }
    }
}

// ---------------------------------------------------------------- chunks (octree)

// Sorts the points into an octree whose leaves hold at most kMaxChunkPoints points.
// Each node splits at the center of the tight bounding box of its own points, so far
// outliers (e.g. garbage coordinates of 1e38) are separated within a few levels.
// Each leaf becomes a chunk: contiguous in the arrays, shuffled, with a tight bounding box.
class ChunkBuilder
{
public:
    ChunkBuilder(CloudData& d, LoadProgress& progress)
        : pos(d.positions.data()), col(d.colors.empty() ? nullptr : d.colors.data()),
          total(d.positions.size()), progress(progress) {}

    std::vector<Chunk> build()
    {
        std::vector<Chunk> chunks;
        node(0, total, 0, chunks);
        return chunks;
    }

private:
    VertexPos* pos;
    Rgba8* col;
    std::size_t total;
    LoadProgress& progress;
    std::atomic<std::size_t> done{0};

    void swapPoints(std::size_t a, std::size_t b)
    {
        std::swap(pos[a], pos[b]);
        if (col) std::swap(col[a], col[b]);
    }

    // Moves points with coordinate < split to the front of [b, e); returns the boundary
    std::size_t partition(std::size_t b, std::size_t e, int axis, float split)
    {
        while (true){
            while (b < e && pos[b].position[axis] < split) ++b;
            while (b < e && !(pos[e - 1].position[axis] < split)) --e;
            if (b >= e) return b;
            swapPoints(b, e - 1);
            ++b;
            --e;
        }
    }

    void bounds(std::size_t b, std::size_t e, glm::vec3& lo, glm::vec3& hi) const
    {
        lo = glm::vec3(INFINITY);
        hi = glm::vec3(-INFINITY);
        for (std::size_t i = b; i < e; i++){
            lo = glm::min(lo, pos[i].position);
            hi = glm::max(hi, pos[i].position);
        }
    }

    void node(std::size_t b, std::size_t e, int depth, std::vector<Chunk>& out)
    {
        checkCancel(progress);
        if (e - b <= kMaxChunkPoints || depth >= kMaxOctreeDepth){
            leaf(b, e, out);
            return;
        }
        glm::vec3 lo, hi;
        bounds(b, e, lo, hi);
        // split into 8 octants at the center: x first, then y, then z
        glm::vec3 c = lo * 0.5f + hi * 0.5f;   // no overflow for coordinates near FLT_MAX
        std::size_t r[9];
        r[0] = b; r[8] = e;
        r[4] = partition(b, e, 0, c.x);
        r[2] = partition(b, r[4], 1, c.y);
        r[6] = partition(r[4], e, 1, c.y);
        r[1] = partition(b, r[2], 2, c.z);
        r[3] = partition(r[2], r[4], 2, c.z);
        r[5] = partition(r[4], r[6], 2, c.z);
        r[7] = partition(r[6], e, 2, c.z);

        // all points identical (or too close to split in float): cut into pieces
        for (int i = 0; i < 8; i++)
            if (r[i + 1] - r[i] == e - b){
                leaf(b, e, out);
                return;
            }

        if (depth < kParallelDepth){
            std::vector<Chunk> childOut[8];
            std::vector<std::future<void>> tasks;
            for (int i = 0; i < 8; i++){
                if (r[i] == r[i + 1]) continue;
                tasks.push_back(std::async(std::launch::async, [this, &r, &childOut, i, depth]{
                    node(r[i], r[i + 1], depth + 1, childOut[i]);
                }));
            }
            for (auto& t : tasks) t.wait();
            for (auto& t : tasks) t.get();   // rethrows LoadCancelled
            for (auto& co : childOut)
                out.insert(out.end(), co.begin(), co.end());
        }
        else {
            for (int i = 0; i < 8; i++){
                if (r[i] == r[i + 1]) continue;
                node(r[i], r[i + 1], depth + 1, out);
            }
        }
    }

    // Leaves that could not be split further (identical points) are cut into pieces
    void leaf(std::size_t b, std::size_t e, std::vector<Chunk>& out)
    {
        for (std::size_t pb = b; pb < e; pb += kMaxChunkPoints){
            std::size_t pe = std::min(e, pb + kMaxChunkPoints);
            // shuffle, so that the first k points of the chunk are an even subsample
            std::minstd_rand rng((unsigned)pb + 1u);
            for (std::size_t i = pe - 1; i > pb; i--){
                std::size_t j = pb + rng() % (i - pb + 1);
                swapPoints(i, j);
            }
            Chunk c;
            c.first = (std::uint32_t)pb;
            c.count = (std::uint32_t)(pe - pb);
            bounds(pb, pe, c.min, c.max);
            out.push_back(c);
            checkCancel(progress);
        }
        std::size_t d = done.fetch_add(e - b) + (e - b);
        progress.fraction = (float)d / (float)total;
    }
};

} // namespace

CloudData loadCloudData(const std::string& filename, LoadProgress& progress)
{
    std::string ext = std::filesystem::path(filename).extension().string();
    for (auto& ch : ext) ch = (char)std::tolower((unsigned char)ch);

    CloudData d;
    d.min = glm::vec3(INFINITY);
    d.max = glm::vec3(-INFINITY);
    progress.stage = "Reading";
    progress.fraction = 0.0f;
    if (ext == ".obj")
        readObj(filename, d, progress);
    else if (ext == ".ply" || ext == ".pcd"){
        bool done = ext == ".ply" ? readPlyBinary(filename, d, progress) : readPcd(filename, d, progress);
        if (!done){
            d = CloudData();
            d.min = glm::vec3(INFINITY);
            d.max = glm::vec3(-INFINITY);
            readWithPcl(filename, ext, d, progress);
        }
    }
    else
        throw std::runtime_error("unsupported file type: " + filename);
    checkCancel(progress);

    if (d.positions.empty())
        throw std::runtime_error("no points in " + filename);
    if (d.positions.size() > 0xFFFFFFFFull)
        throw std::runtime_error("too many points (max. 4.29 billion): " + filename);

    progress.stage = "Sorting into chunks";
    progress.fraction = 0.0f;
    ChunkBuilder builder(d, progress);
    d.chunks = builder.build();
    return d;
}
