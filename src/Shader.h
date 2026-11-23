#pragma once
#include <glad/glad.h>
#include <string>

// OpenGL shader program built from a vertex and a fragment shader file.
// Lines of the form  #include "file.glsl"  are replaced by that file's content
// (looked up next to the including shader).
class Shader
{
public:
	GLuint ID = 0;

	Shader(const std::string& vertexFile, const std::string& fragmentFile);

	void Activate();
	void Delete();

	// Uniform location by name (-1 if the shader does not use it)
	GLint uniform(const char* name) const { return glGetUniformLocation(ID, name); }

private:
	static std::string loadSource(const std::string& file);
	static bool checkErrors(GLuint object, bool isProgram, const std::string& what);
};
