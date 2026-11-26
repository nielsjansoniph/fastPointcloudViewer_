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
        if (ext != ".ply" || !readPlyBinary(filename, d, progress)){
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
