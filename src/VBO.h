#pragma once
#include <glad/glad.h>
#include <glm/glm.hpp>
#include <vector>

struct VertexPos{
	glm::vec3 position;
};

struct VertexPosCol{
	glm::vec3 position;
	glm::vec3 color;
};

// Vertex Buffer Object: uploads vertex data to the GPU (stays bound after construction)
class VBO
{
public:
	GLuint ID = 0;

	VBO(const std::vector<VertexPos>& vertices);
	VBO(const std::vector<VertexPosCol>& vertices);
	VBO(const void* data, GLsizeiptr bytes);

	void Bind();
	void Unbind();
	void Delete();
};
