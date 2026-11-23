#include "Cloud.h"

#include <cctype>
#include <filesystem>
#include <stdexcept>
#include <vector>

#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>

#include "VBO.h"

Cloud::Cloud(const std::string& filename)
	: filename(filename)
{
	std::vector<VertexPos> positions;
	{
		// The PCL cloud only lives in this scope, so its memory is freed before the upload
		pcl::PointCloud<pcl::PointXYZ> tmp;
		std::string ext = std::filesystem::path(filename).extension().string();
		for (auto& ch : ext) ch = (char)std::tolower((unsigned char)ch);

		int result = -1;
		if (ext == ".ply"){
			pcl::PLYReader reader;
			result = reader.read(filename, tmp);
		}
		else if (ext == ".pcd"){
			pcl::PCDReader reader;
			result = reader.read(filename, tmp);
		}
		else
			throw std::runtime_error("unsupported file type: " + filename);

		if (result < 0)
			throw std::runtime_error("could not read " + filename);
		if (tmp.points.empty())
			throw std::runtime_error("no points in " + filename);

		pcl::PointXYZ minPt, maxPt;
		pcl::getMinMax3D(tmp, minPt, maxPt);
		min = glm::vec3(minPt.x, minPt.y, minPt.z);
		max = glm::vec3(maxPt.x, maxPt.y, maxPt.z);

		// Positions only (12 bytes/point); the shaders do not use a per-point color
		positions.resize(tmp.points.size());
		for (size_t i = 0; i < tmp.points.size(); i++)
			positions[i].position = glm::vec3(tmp.points[i].x, tmp.points[i].y, tmp.points[i].z);
	}
	numPoints = (GLsizei)positions.size();

	VAO.Bind();
	VBO vbo(positions);
	VAO.LinkAttrib(vbo, 0, 3, GL_FLOAT, sizeof(VertexPos), (void*)0);
	VAO.Unbind();
	vbo.Unbind();
	// positions goes out of scope here: the data is kept on the GPU only
}

void Cloud::Draw()
{
	VAO.Bind();
	glDrawArrays(GL_POINTS, 0, numPoints);
	VAO.Unbind();
}
