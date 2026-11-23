#ifndef CLOUD_CLASS_H
#define CLOUD_CLASS_H


#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include<string>

#include "VAO.h"
#include "Camera.h"
#include "shaderClass.h"
#include <pcl/io/ply_io.h>

#include <pcl/point_types.h>
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <filesystem>
#include <iostream>
#include <stdexcept>


class Cloud
{
public:
	// Number of points; the point data itself lives only on the GPU
	GLsizei numPoints = 0;
	VertexPosCol min;
	VertexPosCol max;
	// Store VAO in public so it can be used in the Draw function
	::VAO VAO;

	// Initializes the mesh
	Cloud(std::vector<VertexPosCol>& vertices);

	// Initializes with filename to read (.ply or .pcd); throws std::runtime_error on failure
	Cloud(const std::string filename);

	// Draws the mesh
	void Draw(Camera& camera);
	void DrawElements(Camera& camera);

	int shaderType = 0;
	Shader* currentShader;

	glm::vec3 point_color = glm::vec3(1.0f);

	std::string filename;

	float pointssize = 5;
	
};
#endif
