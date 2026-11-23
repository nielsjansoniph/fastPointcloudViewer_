#pragma once
#include <glm/glm.hpp>

#include "Camera.h"
#include "Shader.h"
#include "VAO.h"

// Coordinate axes (X red, Y green, Z blue) with labels, drawn on top of the points
class Axes
{
public:
    Axes();   // needs a current OpenGL context

    bool show = true;
    float length = 1.0f;

    // Draws the axes at `center`; the labels go to ImGui's foreground draw list,
    // so this must be called before ImGui::Render()
    void draw(const Camera& camera, const glm::vec3& center);

private:
    Shader shader;
    ::VAO vao;
};
