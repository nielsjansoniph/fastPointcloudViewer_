#ifndef CLOUD_CLASS_H
#define CLOUD_CLASS_H

#include<string>

#include "VAO.h"
#include "Camera.h"
#include "shaderClass.h"
#include <pcl/io/ply_io.h>
//#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

class Cloud
{
public:
	std::vector<Vertex> vertices;
	// Store VAO in public so it can be used in the Draw function
	VAO VAO;

	// Initializes the mesh
	Cloud(std::vector<Vertex>& vertices);

	// Initializes with filename to read
	Cloud(const std::string filename);

	// Draws the mesh
	void Draw(Camera& camera);

	int shaderType = 0;
	Shader* currentShader;
	
};
#endif
