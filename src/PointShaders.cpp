#include "PointShaders.h"

#include <algorithm>
#include <cfloat>
#include <cmath>
#include <iostream>

#include <glm/gtc/type_ptr.hpp>
#include <imgui.h>

#include "Paths.h"

namespace {

Shader load(const char* vert, const char* frag)
{
    std::cout << "Compiling " << vert << " + " << frag << std::endl;
    return Shader(shaderPath(vert), shaderPath(frag));
}

void setFloat(const Shader& s, const char* name, float v) { glUniform1f(s.uniform(name), v); }
void setVec3(const Shader& s, const char* name, const glm::vec3& v) { glUniform3fv(s.uniform(name), 1, glm::value_ptr(v)); }

// Uniforms of the ray-cast cube/sphere shaders; sizeUniform gets PointSize in cm as world size
void setRaycastUniforms(const Shader& s, const char* sizeUniform, const Cloud& c, const RenderContext& ctx)
{
    glm::mat4 invCam = glm::inverse(ctx.camera.cameraMatrix);
    float projScale = (float)ctx.viewportHeight / std::tan(glm::radians(45.0f) / 2.0f);
    setFloat(s, sizeUniform, c.pointssize * 0.01f);
    setFloat(s, "projScale", projScale);
    glUniform2f(s.uniform("viewportSize"), (float)ctx.viewportWidth, (float)ctx.viewportHeight);
    glUniformMatrix4fv(s.uniform("invCamMatrix"), 1, GL_FALSE, glm::value_ptr(invCam));
}

} // namespace

std::vector<PointShader> createPointShaders()
{
    std::vector<PointShader> shaders;

    // 0: color by distance
    shaders.push_back({"Default", load("points.vert", "default.frag"), false,
        [](Cloud&, ShaderSettings& st, const glm::mat4&){
            ImGui::SliderFloat("CFactor", &st.cFactor, 0.0f, 10.0f);
        },
        [](const Shader& s, const Cloud&, const ShaderSettings& st, const RenderContext&){
            setFloat(s, "cFactor", st.cFactor);
        }});

    // 1: one color (default for new clouds)
    shaders.push_back({"Monocolor", load("points.vert", "monoColor.frag"), false,
        [](Cloud& c, ShaderSettings&, const glm::mat4&){
            ImGui::ColorEdit3("Point color", (float*)&c.point_color);
        },
        [](const Shader& s, const Cloud& c, const ShaderSettings&, const RenderContext&){
            setVec3(s, "color", c.point_color);
        }});

    // 2: axis-aligned cubes
    shaders.push_back({"Cube", load("cube.vert", "cube.frag"), true,
        [](Cloud&, ShaderSettings&, const glm::mat4&){
            ImGui::TextDisabled("Cube edge length = PointSize in cm");
        },
        [](const Shader& s, const Cloud& c, const ShaderSettings&, const RenderContext& ctx){
            setRaycastUniforms(s, "cubeSize", c, ctx);
        }});

    // 3: hue gradient by distance
    shaders.push_back({"ColorGradient", load("points.vert", "colorgradient.frag"), false,
        [](Cloud&, ShaderSettings& st, const glm::mat4&){
            ImGui::SliderFloat("Start distance", &st.startDist, 0.0f, 50.f);
            ImGui::SliderFloat("End distance", &st.endDist, 0.0f, 500.0f);
            ImGui::SliderFloat("Start Brightness", &st.startBrightness, 0.0f, 1.0f);
            ImGui::SliderFloat("End Brightness", &st.endBrightness, 0.0f, 1.0f);
            ImGui::SliderFloat("CFactor", &st.cFactor, 0.0f, 10.0f);
        },
        [](const Shader& s, const Cloud&, const ShaderSettings& st, const RenderContext&){
            setFloat(s, "cFactor", st.cFactor);
            setFloat(s, "startDist", st.startDist);
            setFloat(s, "endDist", st.endDist);
        }});

    // 4: split view, distance coloring
    shaders.push_back({"Debug", load("points.vert", "debug.frag"), false,
        [](Cloud&, ShaderSettings& st, const glm::mat4&){
            ImGui::SliderFloat("Split", &st.split, 0.0f, ImGui::GetIO().DisplaySize.x);
            if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
                ImGui::SetTooltip("Slide to change debug view border");
            ImGui::SliderFloat("Start distance", &st.startDist, 0.0f, 20.0f);
            ImGui::SliderFloat("End distance", &st.endDist, 0.0f, 50.0f);
            ImGui::ColorEdit3("Start color", (float*)&st.startColor);
            ImGui::ColorEdit3("End color", (float*)&st.endColor);
        },
        [](const Shader& s, const Cloud&, const ShaderSettings& st, const RenderContext&){
            setFloat(s, "startDist", st.startDist);
            setFloat(s, "endDist", st.endDist);
            setFloat(s, "split", st.split);
            setVec3(s, "startColor", st.startColor);
            setVec3(s, "endColor", st.endColor);
        }});

    // 5: split view, color by height z
    shaders.push_back({"zMap", load("points.vert", "zmap.frag"), false,
        [](Cloud& c, ShaderSettings& st, const glm::mat4& modelMatrix){
            ImGui::SliderFloat("Split", &st.split, 0.0f, ImGui::GetIO().DisplaySize.x);
            // height (z) range of the cloud after applying the model rotation
            float zMin = FLT_MAX, zMax = -FLT_MAX;
            for (int k = 0; k < 8; k++){
                glm::vec3 corner((k & 1) ? c.max.x : c.min.x,
                                 (k & 2) ? c.max.y : c.min.y,
                                 (k & 4) ? c.max.z : c.min.z);
                float z = (modelMatrix * glm::vec4(corner, 1.0f)).z;
                zMin = std::min(zMin, z);
                zMax = std::max(zMax, z);
            }
            ImGui::SliderFloat("Start height", &st.startDist, zMin, zMax);
            ImGui::SliderFloat("End height", &st.endDist, zMin, zMax);
        },
        [](const Shader& s, const Cloud&, const ShaderSettings& st, const RenderContext&){
            setFloat(s, "startDist", st.startDist);
            setFloat(s, "endDist", st.endDist);
            setFloat(s, "split", st.split);
        }});

    // 6: spheres colored by surface direction
    shaders.push_back({"Sphere", load("sphere.vert", "sphere.frag"), true,
        [](Cloud&, ShaderSettings&, const glm::mat4&){
            ImGui::TextDisabled("Sphere diameter = PointSize in cm");
        },
        [](const Shader& s, const Cloud& c, const ShaderSettings&, const RenderContext& ctx){
            setRaycastUniforms(s, "sphereSize", c, ctx);
        }});

    return shaders;
}
