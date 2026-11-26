#include "Cloud.h"

#include <algorithm>
#include <cctype>
#include <charconv>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <type_traits>
#include <vector>

#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>

#include "VBO.h"

namespace {

struct Rgba8 { std::uint8_t r, g, b, a; };

// Copies positions (and colors) out of a PCL cloud and computes the bounding box
template <typename PointT>
void extract(const pcl::PointCloud<PointT>& cloud, std::vector<VertexPos>& positions,
             glm::vec3& min, glm::vec3& max, std::vector<Rgba8>* colors)
{
    positions.reserve(cloud.points.size());
    if (colors) colors->reserve(cloud.points.size());
    min = glm::vec3(INFINITY);
    max = glm::vec3(-INFINITY);
    for (const auto& p : cloud.points){
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
            continue;   // skip invalid points
        glm::vec3 pos(p.x, p.y, p.z);
        positions.push_back({pos});
        min = glm::min(min, pos);
        max = glm::max(max, pos);
        if constexpr (std::is_same_v<PointT, pcl::PointXYZRGBA>)
            colors->push_back({p.r, p.g, p.b, 255});
    }
}

// Reads the vertices ("v x y z [r g b]") of a Wavefront OBJ file as points; faces, normals
// and texture coordinates are ignored. Vertex colors are taken if every vertex has one
// (range 0..1, or 0..255 if any component is > 1). Points are rotated from Y-up to Z-up.
void readObj(const std::string& filename, std::vector<VertexPos>& positions,
             glm::vec3& min, glm::vec3& max, std::vector<Rgba8>& colors, bool& hasColor)
{
    std::ifstream in(filename, std::ios::binary);
    if (!in)
        throw std::runtime_error("could not read " + filename);

    std::vector<glm::vec3> rgb;     // float colors, converted once the range is known
    bool allColored = true;
    float maxComponent = 0.0f;
    min = glm::vec3(INFINITY);
    max = glm::vec3(-INFINITY);

    std::string line;
    while (std::getline(in, line)){
        if (line.size() < 2 || line[0] != 'v' || (line[1] != ' ' && line[1] != '\t'))
            continue;   // only plain vertices, not vn/vt/f/...
        float val[6];
        int n = 0;
        // from_chars is locale-independent (strtof would expect ',' in a German locale)
        const char* s = line.data() + 2;
        const char* e = line.data() + line.size();
        while (n < 6){
            while (s < e && (*s == ' ' || *s == '\t')) ++s;
            auto [end, ec] = std::from_chars(s, e, val[n]);
            if (ec != std::errc()) break;
            ++n;
            s = end;
        }
        if (n < 3)
            continue;
        // OBJ is Y-up, the viewer is Z-up: rotate +90° about X, (x, y, z) -> (x, -z, y)
        glm::vec3 pos(val[0], -val[2], val[1]);
        if (!std::isfinite(pos.x) || !std::isfinite(pos.y) || !std::isfinite(pos.z))
            continue;
        positions.push_back({pos});
        min = glm::min(min, pos);
        max = glm::max(max, pos);
        if (n == 6 && allColored){
            rgb.emplace_back(val[3], val[4], val[5]);
            maxComponent = std::max({maxComponent, val[3], val[4], val[5]});
        }
        else
            allColored = false;
    }

    hasColor = allColored && !rgb.empty() && rgb.size() == positions.size();
    if (hasColor){
        float scale = maxComponent > 1.0f ? 1.0f : 255.0f;
        colors.reserve(rgb.size());
        for (const auto& c : rgb){
            glm::vec3 v = glm::clamp(c * scale, 0.0f, 255.0f) + 0.5f;
            colors.push_back({(std::uint8_t)v.r, (std::uint8_t)v.g, (std::uint8_t)v.b, 255});
        }
    }
}

} // namespace

Cloud::Cloud(const std::string& filename)
	: filename(filename)
{
	std::vector<VertexPos> positions;
	std::vector<Rgba8> colors;
	std::string ext = std::filesystem::path(filename).extension().string();
	for (auto& ch : ext) ch = (char)std::tolower((unsigned char)ch);
	if (ext == ".obj")
		readObj(filename, positions, min, max, colors, hasColor);
	else {
		// Read the file once into a generic cloud, then convert depending on the fields.
		// Everything in this scope is freed before the GPU upload.
		pcl::PCLPointCloud2 blob;

		int result = -1;
		if (ext == ".ply"){
			pcl::PLYReader reader;
			result = reader.read(filename, blob);
		}
		else if (ext == ".pcd"){
			pcl::PCDReader reader;
			result = reader.read(filename, blob);
		}
		else
			throw std::runtime_error("unsupported file type: " + filename);
		if (result < 0)
			throw std::runtime_error("could not read " + filename);

		for (const auto& field : blob.fields)
			if (field.name == "rgb" || field.name == "rgba")
				hasColor = true;

		if (hasColor){
			pcl::PointCloud<pcl::PointXYZRGBA> cloud;
			pcl::fromPCLPointCloud2(blob, cloud);
			blob = pcl::PCLPointCloud2();   // free the raw data early
			extract(cloud, positions, min, max, &colors);
		}
		else{
			pcl::PointCloud<pcl::PointXYZ> cloud;
			pcl::fromPCLPointCloud2(blob, cloud);
			blob = pcl::PCLPointCloud2();
			extract(cloud, positions, min, max, (std::vector<Rgba8>*)nullptr);
		}
	}
	if (positions.empty())
		throw std::runtime_error("no points in " + filename);
	numPoints = (GLsizei)positions.size();

	// Positions (12 bytes/point) and, if present, colors (4 bytes/point) in separate buffers
	VAO.Bind();
	VBO positionBuffer(positions);
	VAO.LinkAttrib(positionBuffer, 0, 3, GL_FLOAT, sizeof(VertexPos), (void*)0);
	if (hasColor){
		VBO colorBuffer(colors.data(), colors.size() * sizeof(Rgba8));
		VAO.LinkAttrib(colorBuffer, 1, 4, GL_UNSIGNED_BYTE, sizeof(Rgba8), (void*)0, GL_TRUE);
	}
	VAO.Unbind();
	positionBuffer.Unbind();
	// positions/colors go out of scope here: the data is kept on the GPU only
}

void Cloud::Draw()
{
	if (!hasColor)
		glVertexAttrib4f(1, 1.0f, 1.0f, 1.0f, 1.0f);   // shaders reading the color get white
	VAO.Bind();
	glDrawArrays(GL_POINTS, 0, numPoints);
	VAO.Unbind();
}
