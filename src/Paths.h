#pragma once
// Locations of runtime files. FPCV_SOURCE_DIR is set by CMake to the project
// directory, so fpcv can be started from any working directory.
#include <cstdlib>
#include <string>

#ifndef FPCV_SOURCE_DIR
#define FPCV_SOURCE_DIR "."
#endif

// Shader file; the environment variable FPCV_SHADER_DIR overrides the shader folder
inline std::string shaderPath(const std::string& name)
{
    const char* dir = std::getenv("FPCV_SHADER_DIR");
    return (dir ? std::string(dir) : std::string(FPCV_SOURCE_DIR) + "/shaders") + "/" + name;
}

// Folder for recorded videos
inline std::string recordsDir()
{
    return std::string(FPCV_SOURCE_DIR) + "/records";
}
