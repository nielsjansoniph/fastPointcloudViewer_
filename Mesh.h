#ifndef MESH_CLASS_H
#define MESH_CLASS_H

#include<string>

#include"VAO.h"
#include"EBO.h"
#include"Camera.h"
#include"Texture.h"

class Mesh
{
public:
	std::vector <VertexPosNorCol> vertices;
	std::vector <GLuint> indices;
	std::vector <Texture> textures;
	// Store VAO in public so it can be used in the Draw function
	VAO VAO;
	// Initializes the mesh
	Mesh(std::vector <VertexPosNorCol>& vertices, std::vector<GLuint>& indices);

    Shader* currentShader;

	// Draws the mesh
	void Draw(Shader& shader, Camera& camera);

    void Draw();
};
#endif