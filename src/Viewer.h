#pragma once
// The viewer application: owns camera, clouds, shaders and UI state and runs one frame at a time
#include <deque>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <glad/glad.h>   // must come before GLFW
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <imgui.h>

#include "Axes.h"
#include "Camera.h"
#include "CameraPath.h"
#include "Cloud.h"
#include "PointShaders.h"

struct LoadJob;   // a file being loaded on the background thread (Viewer.cpp)

class Viewer
{
public:
    // Needs a current OpenGL context and an initialized ImGui
    explicit Viewer(GLFWwindow* window);
    ~Viewer();

    // Queues a .ply/.pcd/.obj file for loading in the background (progress window while it
    // loads); prints a message and returns false if the file does not exist or is unsupported
    bool loadCloud(const std::string& path);

    // Processes input, draws the UI and renders one frame (without swapping buffers)
    void frame();

    // Recording/replay needs the frame captured before the buffer swap
    void afterRender() { path.afterRender(ImGui::GetIO().DeltaTime); }

private:
    GLFWwindow* window;
    Camera camera;
    std::vector<Cloud> clouds;
    std::vector<PointShader> pointShaders;
    ShaderSettings settings;
    Axes axes;
    CameraPath path;
    ImVec4 clearColor = ImVec4(0.0f, 0.0f, 0.0f, 1.0f);
    bool loadRequested = false;

    // Background loading, one file after the other
    std::deque<std::string> loadQueue;
    std::shared_ptr<LoadJob> loadJob;
    std::thread loadThread;

    // Level of detail (see Cloud::prepare)
    bool lodEnabled = true;
    float pointBudgetMillions = 30.0f;   // at most this many points per frame
    float lodDetail = 2.0f;              // points per pixel of a chunk's projected size

    // Model orientation: rotated in 90 degree steps around the center of all clouds
    glm::mat4 modelRotation = glm::mat4(1.0f);
    glm::mat4 modelMatrix = glm::mat4(1.0f);
    glm::vec3 modelCenter = glm::vec3(0.0f);

    void drawUi();
    void drawOrientationUi();
    void drawCloudUi(Cloud& cloud);
    void drawLodUi();
    void pollLoading();
    void drawLoadingUi();
    void openLoadDialog();
    void handleInput();
    void updateModelMatrix();
    void render(int framebufferWidth, int framebufferHeight);
};
