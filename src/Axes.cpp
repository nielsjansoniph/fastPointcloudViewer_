#include "Axes.h"

#include <vector>

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <imgui.h>

#include "Paths.h"

Axes::Axes()
    : shader(shaderPath("axis.vert"), shaderPath("axis.frag"))
{
    // unit axes, scaled by `length` when drawing
    std::vector<VertexPosCol> vertices = {
        {{0.0f, 0.0f, 0.0f}, {1.0f, 0.2f, 0.2f}}, {{1.0f, 0.0f, 0.0f}, {1.0f, 0.2f, 0.2f}},
        {{0.0f, 0.0f, 0.0f}, {0.2f, 1.0f, 0.2f}}, {{0.0f, 1.0f, 0.0f}, {0.2f, 1.0f, 0.2f}},
        {{0.0f, 0.0f, 0.0f}, {0.3f, 0.5f, 1.0f}}, {{0.0f, 0.0f, 1.0f}, {0.3f, 0.5f, 1.0f}},
    };
    vao.Bind();
    VBO vbo(vertices);
    vao.LinkAttrib(vbo, 0, 3, GL_FLOAT, sizeof(VertexPosCol), (void*)0);
    vao.LinkAttrib(vbo, 1, 3, GL_FLOAT, sizeof(VertexPosCol), (void*)(3 * sizeof(float)));
    vao.Unbind();
    vbo.Unbind();
}

void Axes::draw(const Camera& camera, const glm::vec3& center)
{
    if (!show)
        return;

    glm::mat4 model = glm::translate(glm::mat4(1.0f), center) * glm::scale(glm::mat4(1.0f), glm::vec3(length));
    shader.Activate();
    glUniformMatrix4fv(shader.uniform("camMatrix"), 1, GL_FALSE, glm::value_ptr(camera.cameraMatrix));
    glUniformMatrix4fv(shader.uniform("model"), 1, GL_FALSE, glm::value_ptr(model));
    glDisable(GL_DEPTH_TEST);   // always on top of the points
    vao.Bind();
    glLineWidth(3.0f);
    glDrawArrays(GL_LINES, 0, 6);
    glLineWidth(1.0f);
    vao.Unbind();
    glEnable(GL_DEPTH_TEST);

    // Labels at the axis tips, projected to screen coordinates
    ImDrawList* drawList = ImGui::GetForegroundDrawList();
    ImVec2 displaySize = ImGui::GetIO().DisplaySize;
    const char* labels[3] = {"X", "Y", "Z"};
    const ImU32 colors[3] = {IM_COL32(255, 60, 60, 255), IM_COL32(60, 255, 60, 255), IM_COL32(90, 140, 255, 255)};
    float fontSize = ImGui::GetFontSize() * 1.5f;
    for (int a = 0; a < 3; a++){
        glm::vec3 tip = center;
        tip[a] += length * 1.1f;
        glm::vec4 clip = camera.cameraMatrix * glm::vec4(tip, 1.0f);
        if (clip.w <= 0.0f) continue;   // behind the camera
        glm::vec3 ndc = glm::vec3(clip) / clip.w;
        ImVec2 p((ndc.x * 0.5f + 0.5f) * displaySize.x, (0.5f - ndc.y * 0.5f) * displaySize.y);
        drawList->AddText(ImGui::GetFont(), fontSize, ImVec2(p.x - fontSize * 0.25f, p.y - fontSize * 0.5f), colors[a], labels[a]);
    }
}
