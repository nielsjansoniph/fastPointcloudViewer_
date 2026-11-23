#include "Shader.h"

#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>

// Reads a shader file and resolves #include "..." lines
std::string Shader::loadSource(const std::string& file)
{
	std::ifstream in(file, std::ios::binary);
	if (!in)
		throw std::runtime_error("cannot open shader file " + file);

	std::filesystem::path dir = std::filesystem::path(file).parent_path();
	std::stringstream out;
	std::string line;
	while (std::getline(in, line)){
		if (!line.empty() && line.back() == '\r')
			line.pop_back();
		const std::string directive = "#include";
		size_t first = line.find_first_not_of(" \t");
		if (first != std::string::npos && line.compare(first, directive.size(), directive) == 0){
			size_t q1 = line.find('"', first);
			size_t q2 = line.find('"', q1 + 1);
			if (q1 == std::string::npos || q2 == std::string::npos)
				throw std::runtime_error("malformed #include in " + file + ": " + line);
			out << loadSource((dir / line.substr(q1 + 1, q2 - q1 - 1)).string()) << "\n";
		}
		else
			out << line << "\n";
	}
	return out.str();
}

Shader::Shader(const std::string& vertexFile, const std::string& fragmentFile)
{
	std::string vertexCode = loadSource(vertexFile);
	std::string fragmentCode = loadSource(fragmentFile);
	const char* vertexSource = vertexCode.c_str();
	const char* fragmentSource = fragmentCode.c_str();

	GLuint vertexShader = glCreateShader(GL_VERTEX_SHADER);
	glShaderSource(vertexShader, 1, &vertexSource, NULL);
	glCompileShader(vertexShader);
	checkErrors(vertexShader, false, vertexFile);

	GLuint fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
	glShaderSource(fragmentShader, 1, &fragmentSource, NULL);
	glCompileShader(fragmentShader);
	checkErrors(fragmentShader, false, fragmentFile);

	ID = glCreateProgram();
	glAttachShader(ID, vertexShader);
	glAttachShader(ID, fragmentShader);
	glLinkProgram(ID);
	checkErrors(ID, true, vertexFile + " + " + fragmentFile);

	glDeleteShader(vertexShader);
	glDeleteShader(fragmentShader);
}

void Shader::Activate()
{
	glUseProgram(ID);
}

void Shader::Delete()
{
	glDeleteProgram(ID);
}

// Prints compile/link errors; returns true if everything is fine
bool Shader::checkErrors(GLuint object, bool isProgram, const std::string& what)
{
	GLint ok;
	char infoLog[1024];
	if (isProgram){
		glGetProgramiv(object, GL_LINK_STATUS, &ok);
		if (ok == GL_FALSE){
			glGetProgramInfoLog(object, 1024, NULL, infoLog);
			std::cout << "SHADER_LINKING_ERROR for: " << what << "\n" << infoLog << std::endl;
		}
	}
	else{
		glGetShaderiv(object, GL_COMPILE_STATUS, &ok);
		if (ok == GL_FALSE){
			glGetShaderInfoLog(object, 1024, NULL, infoLog);
			std::cout << "SHADER_COMPILATION_ERROR for: " << what << "\n" << infoLog << std::endl;
		}
	}
	return ok != GL_FALSE;
}
