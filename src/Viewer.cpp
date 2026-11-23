#include "Viewer.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <filesystem>
#include <iostream>

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <nfd.h>

#include "UiWidgets.h"

namespace {
constexpr float kFovDeg = 45.0f;
constexpr float kNearPlane = 0.1f;
constexpr float kFarPlane = 500.0f;
}

Viewer::Viewer(GLFWwindow* window)
    : window(window),
      camera(1600, 900, glm::vec3(0.0f, -2.0f, 0.0f)),
      pointShaders(createPointShaders())
{
    glEnable(GL_DEPTH_TEST);
}

bool Viewer::loadCloud(const std::string& path)
{
    std::string ext = std::filesystem::path(path).extension().string();
    std::transform(ext.begin(), ext.end(), ext.begin(), [](unsigned char c){ return (char)std::tolower(c); });
    if (!std::filesystem::is_regular_file(path)){
        std::cerr << "File not found: " << path << std::endl;
        return false;
    }
    if (ext != ".ply" && ext != ".pcd"){
        std::cerr << "Unsupported file type (only .ply and .pcd): " << path << std::endl;
        return false;
    }
    std::cout << "Loading " << path << std::endl;
    try {
        clouds.emplace_back(path);
    }
    catch (const std::exception& e) {
        std::cerr << "Error loading point cloud: " << e.what() << std::endl;
        return false;
    }
    clouds.back().shaderType = kDefaultPointShader;
    std::cout << "Loaded " << clouds.back().numPoints << " points" << std::endl;
    return true;
}

void Viewer::frame()
{
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    drawUi();
    if (loadRequested){
        loadRequested = false;
        openLoadDialog();
    }

    int fbWidth, fbHeight, winWidth, winHeight;
    glfwGetFramebufferSize(window, &fbWidth, &fbHeight);
    glfwGetWindowSize(window, &winWidth, &winHeight);
    // camera uses the window size for the aspect ratio and for centering the mouse
    if (winWidth > 0 && winHeight > 0){
        camera.width = winWidth;
        camera.height = winHeight;
    }

    if (!path.isReplaying())
        handleInput();
    path.update(camera, fbWidth, fbHeight);
    camera.updateMatrix(kFovDeg, kNearPlane, kFarPlane);

    render(fbWidth, fbHeight);
}

// ---------------------------------------------------------------- UI

void Viewer::drawUi()
{
    ImGui::Begin("Viewer settings");
    SliderFloatWithButtons("Near", &camera.nearDist, 0.0f, 1.0f);
    SliderFloatWithButtons("Far", &camera.farDist, 0.0f, 100.0f);
    SliderFloatWithButtons("Speed (m/s)", &camera.speed, 0.0f, 50.0f, 1.25f, "%.2f", ImGuiSliderFlags_Logarithmic);
    ImGui::Text("Background color");
    ImGui::ColorEdit3("##background", (float*)&clearColor);
    ImGui::Checkbox("Use depth for point brightness", &camera.useDepthOnPointBrightness);
    ImGui::Checkbox("Use shadow", &camera.useShadow);

    drawOrientationUi();

    loadRequested = ImGui::Button("Load");
    for (size_t i = 0; i < clouds.size(); i++){
        ImGui::PushID((int)i);
        drawCloudUi(clouds[i]);
        ImGui::PopID();
    }

    ImGuiIO& io = ImGui::GetIO();
    ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / io.Framerate, io.Framerate);
    path.drawUi(camera);
    ImGui::End();
}

void Viewer::drawOrientationUi()
{
    if (!ImGui::CollapsingHeader("Orientation", ImGuiTreeNodeFlags_DefaultOpen))
        return;

    ImGui::Checkbox("Show axes", &axes.show);
    ImGui::SliderFloat("Axis length", &axes.length, 0.1f, 100.0f, "%.1f", ImGuiSliderFlags_Logarithmic);
    const char* axisNames[3] = {"X", "Y", "Z"};
    for (int a = 0; a < 3; a++){
        ImGui::PushID(a);
        glm::vec3 axis(0.0f);
        axis[a] = 1.0f;
        float angle = 0.0f;
        ImGui::Text("Rotate around %s", axisNames[a]);
        ImGui::SameLine(130);
        if (ImGui::Button("-90")) angle = -90.0f;
        ImGui::SameLine();
        if (ImGui::Button("+90")) angle = 90.0f;
        if (angle != 0.0f){
            modelRotation = glm::rotate(glm::mat4(1.0f), glm::radians(angle), axis) * modelRotation;
            // 90 degree rotations only contain -1, 0 and 1: round away float drift
            for (int col = 0; col < 3; col++)
                for (int row = 0; row < 3; row++)
                    modelRotation[col][row] = std::round(modelRotation[col][row]);
        }
        ImGui::PopID();
    }
    if (ImGui::Button("Reset rotation"))
        modelRotation = glm::mat4(1.0f);
    ImGui::Separator();
}

void Viewer::drawCloudUi(Cloud& cloud)
{
    ImGui::TextUnformatted(cloud.filename.c_str());
    ImGui::SliderFloat("PointSize", &cloud.pointssize, 1, 20);
    for (int s = 0; s < (int)pointShaders.size(); s++){
        if (s % 2 == 1)
            ImGui::SameLine(180);
        ImGui::RadioButton(pointShaders[s].name.c_str(), &cloud.shaderType, s);
    }
    cloud.shaderType = std::clamp(cloud.shaderType, 0, (int)pointShaders.size() - 1);
    pointShaders[cloud.shaderType].drawOptions(cloud, settings, modelMatrix);
    ImGui::NewLine();
}

void Viewer::openLoadDialog()
{
    nfdu8char_t* outPath = nullptr;
    nfdu8filteritem_t filters[1] = {{"point cloud", "pcd,ply"}};
    nfdopendialogu8args_t args = {0};
    args.filterList = filters;
    args.filterCount = 1;
    nfdresult_t result = NFD_OpenDialogU8_With(&outPath, &args);
    if (result == NFD_OKAY){
        loadCloud(outPath);
        NFD_FreePathU8(outPath);
    }
    else if (result == NFD_ERROR)
        std::cerr << "File dialog error: " << NFD_GetError() << std::endl;
}

// ---------------------------------------------------------------- input

void Viewer::handleInput()
{
    // keyboard and mouse only act on the 3D view, not while the mouse is over the menu
    if (ImGui::GetIO().WantCaptureMouse)
        return;

    float dt = ImGui::GetIO().DeltaTime;   // movement and rotation independent of the frame rate
    if (ImGui::IsKeyDown(ImGuiKey_W)) camera.forward(dt);
    if (ImGui::IsKeyDown(ImGuiKey_S)) camera.backward(dt);
    if (ImGui::IsKeyDown(ImGuiKey_A)) camera.left(dt);
    if (ImGui::IsKeyDown(ImGuiKey_D)) camera.right(dt);
    if (ImGui::IsKeyDown(ImGuiKey_R)) camera.up(dt);
    if (ImGui::IsKeyDown(ImGuiKey_F)) camera.down(dt);

    // arrow keys: look around (alternative to the mouse)
    float keyRot = camera.keyRotSpeed * dt;
    if (ImGui::IsKeyDown(ImGuiKey_LeftArrow))  camera.rotate(-keyRot, 0.0f);
    if (ImGui::IsKeyDown(ImGuiKey_RightArrow)) camera.rotate(keyRot, 0.0f);
    if (ImGui::IsKeyDown(ImGuiKey_UpArrow))    camera.rotate(0.0f, keyRot);
    if (ImGui::IsKeyDown(ImGuiKey_DownArrow))  camera.rotate(0.0f, -keyRot);

    if (ImGui::IsKeyDown(ImGuiKey_C))
        path.addWaypointIfChanged(camera);

    camera.Inputs(window);   // mouse look
}

// ---------------------------------------------------------------- rendering

void Viewer::updateModelMatrix()
{
    // pivot: center of the bounding box of all loaded clouds
    if (!clouds.empty()){
        glm::vec3 lo = clouds[0].min;
        glm::vec3 hi = clouds[0].max;
        for (const auto& c : clouds){
            lo = glm::min(lo, c.min);
            hi = glm::max(hi, c.max);
        }
        modelCenter = (lo + hi) * 0.5f;
    }
    modelMatrix = glm::translate(glm::mat4(1.0f), modelCenter) * modelRotation * glm::translate(glm::mat4(1.0f), -modelCenter);
}

void Viewer::render(int framebufferWidth, int framebufferHeight)
{
    glViewport(0, 0, framebufferWidth, framebufferHeight);
    glClearColor(clearColor.x, clearColor.y, clearColor.z, clearColor.w);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    updateModelMatrix();
    RenderContext ctx{camera, modelMatrix, framebufferWidth, framebufferHeight};

    for (auto& c : clouds){
        PointShader& ps = pointShaders[c.shaderType];
        ps.shader.Activate();
        camera.Matrix(ps.shader, "camMatrix");   // camera matrix + common uniforms (near, far, ...)
        glUniformMatrix4fv(ps.shader.uniform("model"), 1, GL_FALSE, glm::value_ptr(modelMatrix));
        ps.setUniforms(ps.shader, c, settings, ctx);

        if (ps.programPointSize)
            glEnable(GL_PROGRAM_POINT_SIZE);
        else
            glPointSize(c.pointssize);
        c.Draw();
        glDisable(GL_PROGRAM_POINT_SIZE);
    }

    // axes and menu are hidden while a tracking shot is replayed/recorded
    bool showOverlay = !path.isReplaying();
    if (showOverlay)
        axes.draw(camera, modelCenter);

    ImGui::Render();
    if (showOverlay)
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
}
