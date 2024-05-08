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