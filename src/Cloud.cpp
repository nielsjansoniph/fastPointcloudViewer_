#include "Cloud.h"

#include <cctype>
#include <cmath>
#include <cstdint>
#include <filesystem>
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

} // namespace

Cloud::Cloud(const std::string& filename)
	: filename(filename)
{
	std::vector<VertexPos> positions;
	std::vector<Rgba8> colors;
	{
		// Read the file once into a generic cloud, then convert depending on the fields.
		// Everything in this scope is freed before the GPU upload.
		pcl::PCLPointCloud2 blob;
		std::string ext = std::filesystem::path(filename).extension().string();
		for (auto& ch : ext) ch = (char)std::tolower((unsigned char)ch);

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
