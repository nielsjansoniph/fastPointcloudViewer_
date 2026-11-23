#pragma once
// All point cloud shaders in one table. To add a shader, add one entry in
// createPointShaders() (PointShaders.cpp) – menu and rendering use the table.
#include <functional>
#include <string>
#include <vector>

#include <glm/glm.hpp>

#include "Camera.h"
#include "Cloud.h"
#include "Shader.h"

// Parameters shared by several shaders (set in the menu, global for all clouds)
struct ShaderSettings
{
    float cFactor = 0.5f;
    float startDist = 2.0f;
    float endDist = 100.0f;
    float startBrightness = 0.8f;   // shown for ColorGradient, not used by the shader yet
    float endBrightness = 0.2f;     // shown for ColorGradient, not used by the shader yet
    float split = 800.0f;           // screen x of the split line (debug, zMap)
    glm::vec3 startColor = glm::vec3(0.5f, 1.0f, 0.5f);
    glm::vec3 endColor = glm::vec3(0.0f, 0.0f, 0.4f);
};

// Per-frame information the shaders may need
struct RenderContext
{
    const Camera& camera;
    glm::mat4 modelMatrix;
    int viewportWidth;
    int viewportHeight;
};

struct PointShader
{
    std::string name;
    Shader shader;
    // true: the vertex shader sets gl_PointSize itself (cube/sphere)
    bool programPointSize = false;
    // Shader specific options below the radio buttons of a cloud
    std::function<void(Cloud&, ShaderSettings&, const glm::mat4& modelMatrix)> drawOptions;
    // Shader specific uniforms (common camera uniforms and "model" are set by the caller)
    std::function<void(const Shader&, const Cloud&, const ShaderSettings&, const RenderContext&)> setUniforms;
};

// Index of the shader new clouds start with (Monocolor)
constexpr int kDefaultPointShader = 1;

// Compiles all point shaders (needs a current OpenGL context)
std::vector<PointShader> createPointShaders();
