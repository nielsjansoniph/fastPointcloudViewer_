#include "Cloud.h"

Cloud::Cloud(std::vector <Vertex>& vertices)
{
	Cloud::vertices = vertices;

	VAO.Bind();
	// Generates Vertex Buffer Object and links it to vertices
	VBO VBO(vertices);
	// Links VBO attributes such as coordinates and colors to VAO
    VAO.LinkAttrib(VBO, 0, 3, GL_FLOAT, sizeof(Vertex), (void*)0);
    VAO.LinkAttrib(VBO, 1, 3, GL_FLOAT, sizeof(Vertex), (void*)(3 * sizeof(float)));
    // Unbind all to prevent accidentally modifying them
	VAO.Unbind();
	VBO.Unbind();
}

Cloud::Cloud(const std::string filename){
	pcl::PointCloud<pcl::PointXYZ>::Ptr tmp (new pcl::PointCloud<pcl::PointXYZ>);
	Cloud::filename = filename;
    pcl::PLYReader Reader;
    if (Reader.read(filename, *tmp)==-1){
        PCL_ERROR ("Couldn't open file :("); 
    }

	int n = tmp->points.size();
    std::vector<Vertex> vertices;
    Vertex vertex;

    for (int i=0; i<n; i++){
        vertex.position[0] = tmp->points[i].x;
        vertex.position[1] = tmp->points[i].y;
        vertex.position[2] = tmp->points[i].z;
        vertex.color[0] = 1.0f; //(coordx[i] - *mmx.first) / (*mmx.second - *mmx.first)/2.0f;
        vertex.color[1] = 1.0f; //(coordy[i] - *mmy.first) / (*mmy.second - *mmy.first)/2.0f;
        vertex.color[2] = 1.0f;
        vertices.push_back(vertex);
    }

	Cloud::vertices = vertices;

	VAO.Bind();
	// Generates Vertex Buffer Object and links it to vertices
	VBO VBO(vertices);
	// Links VBO attributes such as coordinates and colors to VAO
    VAO.LinkAttrib(VBO, 0, 3, GL_FLOAT, sizeof(Vertex), (void*)0);
    VAO.LinkAttrib(VBO, 1, 3, GL_FLOAT, sizeof(Vertex), (void*)(3 * sizeof(float)));
    // Unbind all to prevent accidentally modifying them
	VAO.Unbind();
	VBO.Unbind();
}


void Cloud::Draw(Camera& camera)
{
	// Bind shader to be able to access uniforms
	currentShader->Activate();
	//shader.Activate();
	VAO.Bind();

	// Keep track of how many of each type of textures we have
	
	// Take care of the camera Matrix
	glUniform3f(glGetUniformLocation(currentShader->ID, "camPos"), camera.Position.x, camera.Position.y, camera.Position.z);
	//camera.Matrix(shader, "camMatrix");
    camera.Matrix(*currentShader, "camMatrix");

	// Draw the actual mesh
	glDrawArrays(GL_POINTS, 0, vertices.size());
	//glDrawElements(GL_TRIANGLES, indices.size(), GL_UNSIGNED_INT, 0);
}