#pragma once
#include <glm/glm.hpp>
#include <string>

#include "VAO.h"

// A point cloud loaded from .ply or .pcd. The points live only on the GPU;
// the CPU side keeps the bounding box and the display settings.
class Cloud
{
public:
	// Loads the file; throws std::runtime_error on failure
	explicit Cloud(const std::string& filename);

	// Draws all points (the shader must already be active and set up)
	void Draw();

	std::string filename;
	GLsizei numPoints = 0;
	bool hasColor = false;             // file contains rgb(a) colors (uploaded as vertex attribute 1)
	glm::vec3 min = glm::vec3(0.0f);   // bounding box in file coordinates
	glm::vec3 max = glm::vec3(0.0f);

	// Display settings
	int shaderType = 1;                // index into the point shader list (see PointShaders.h)
	float pointssize = 5;              // pixels; cube/sphere shader: size in cm
	glm::vec3 point_color = glm::vec3(1.0f);

private:
	::VAO VAO;
};
