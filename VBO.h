#ifndef VBO_CLASS_H
#define VBO_CLASS_H

#include<glad/glad.h>
//#include <GLEW/glew.h>
#include<glm/glm.hpp>
#include<vector>

struct VertexPos{
	glm::vec3 position;
};

struct VertexPosCol{
	glm::vec3 position;
	glm::vec3 color;
};

struct VertexPosNorCol
{
	glm::vec3 position;
	glm::vec3 normal;
	glm::vec2 color;
};

struct VertexPosNorColTex{
	glm::vec3 position;
	glm::vec3 normal;
	glm::vec3 color;
	glm::vec2 texUV;
};

class VBO
{
public:
	// Reference ID of the Vertex Buffer Object
	GLuint ID;
	// Constructor that generates a Vertex Buffer Object and links it to vertices

	VBO(std::vector<VertexPos>& vertices);
	VBO(std::vector<VertexPosCol>& vertices);
	VBO(std::vector<VertexPosNorCol>& vertices);
	VBO(std::vector<VertexPosNorColTex>& vertices);

	VBO(GLfloat* vertices, GLsizeiptr size);

	VBO();

	// Binds the VBO
	void Bind();
	// Unbinds the VBO
	void Unbind();
	// Deletes the VBO
	void Delete();
};

#endif