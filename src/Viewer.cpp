#include "Viewer.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <atomic>
#include <chrono>
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

// A file being loaded on the background thread. Shared with the thread, so the thread can
// be detached on exit while it is still inside PCL (which cannot be cancelled).
struct LoadJob
{
    std::string path;
    LoadProgress progress;
    std::atomic<bool> done{false};
    CloudData data;
    std::string error;
    bool cancelled = false;
    std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
};

Viewer::~Viewer()
{
    if (loadThread.joinable()){
        loadJob->progress.cancel = true;
        // own readers react within milliseconds; PCL (unknown progress) does not
        if (loadJob->progress.fraction.load() >= 0.0f)
            loadThread.join();
        else
            loadThread.detach();
    }
}

bool Viewer::loadCloud(const std::string& path)
{
    std::string ext = std::filesystem::path(path).extension().string();
    std::transform(ext.begin(), ext.end(), ext.begin(), [](unsigned char c){ return (char)std::tolower(c); });
    if (!std::filesystem::is_regular_file(path)){
        std::cerr << "File not found: " << path << std::endl;
        return false;
    }
    if (ext != ".ply" && ext != ".pcd" && ext != ".obj"){
        std::cerr << "Unsupported file type (only .ply, .pcd and .obj): " << path << std::endl;
        return false;
    }
    loadQueue.push_back(path);
    return true;
}

void Viewer::pollLoading()
{
    // finished: upload to the GPU (here on the main thread, which owns the OpenGL context)
    if (loadJob && loadJob->done.load(std::memory_order_acquire)){
        loadThread.join();
        LoadJob& job = *loadJob;
        double seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - job.start).count();
        if (job.cancelled)
            std::cout << "Loading cancelled: " << job.path << std::endl;
        else if (!job.error.empty())
            std::cerr << "Error loading point cloud: " << job.error << std::endl;
        else {
            std::size_t chunks = job.data.chunks.size();
            clouds.emplace_back(job.path, std::move(job.data));
            Cloud& cloud = clouds.back();
            cloud.shaderType = cloud.hasColor ? kColorPointShader : kDefaultPointShader;
            std::cout << "Loaded " << cloud.numPoints << " points" << (cloud.hasColor ? " with colors" : "")
                      << " in " << chunks << " chunks, " << seconds << " s" << std::endl;
        }
        loadJob.reset();
    }

    if (!loadJob && !loadQueue.empty()){
        loadJob = std::make_shared<LoadJob>();
        loadJob->path = loadQueue.front();
        loadQueue.pop_front();
        std::cout << "Loading " << loadJob->path << std::endl;
        loadThread = std::thread([job = loadJob]{
            try {
                job->data = loadCloudData(job->path, job->progress);
            }
            catch (const LoadCancelled&) {
                job->cancelled = true;
            }
            catch (const std::exception& e) {
                job->error = e.what();
            }
            catch (...) {
                job->error = "unknown error";
            }
            job->done.store(true, std::memory_order_release);
        });
    }
}

void Viewer::drawLoadingUi()
{
    if (!loadJob)
        return;
    const LoadProgress& p = loadJob->progress;
    ImGuiIO& io = ImGui::GetIO();
    ImGui::SetNextWindowPos(ImVec2(io.DisplaySize.x * 0.5f, io.DisplaySize.y * 0.5f), ImGuiCond_Always, ImVec2(0.5f, 0.5f));
    ImGui::Begin("Loading", nullptr, ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoCollapse
                                     | ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoMove);
    ImGui::TextUnformatted(std::filesystem::path(loadJob->path).filename().string().c_str());
    double seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - loadJob->start).count();
    ImGui::Text("%s  (%.0f s)", p.stage.load(), seconds);
    float fraction = p.fraction.load();
    // negative fraction: ImGui shows an indeterminate (moving) bar
    ImGui::ProgressBar(fraction >= 0.0f ? fraction : -(float)ImGui::GetTime(), ImVec2(400.0f, 0.0f));
    if (!loadQueue.empty())
        ImGui::Text("%d more file(s) queued", (int)loadQueue.size());
    if (p.cancel.load())
        ImGui::TextDisabled("Cancelling...");
    else if (ImGui::Button("Cancel"))
        loadJob->progress.cancel = true;
    ImGui::End();
}

void Viewer::frame()
{
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    drawUi();
    pollLoading();
    drawLoadingUi();
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
    drawLodUi();
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

void Viewer::drawLodUi()
{
    if (!ImGui::CollapsingHeader("Level of detail", ImGuiTreeNodeFlags_DefaultOpen))
        return;
    ImGui::Checkbox("Level of detail", &lodEnabled);
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Off: all points of the visible chunks (always off while recording)");
    ImGui::BeginDisabled(!lodEnabled);
    ImGui::SliderFloat("Point budget (M)", &pointBudgetMillions, 1.0f, 500.0f, "%.0f", ImGuiSliderFlags_Logarithmic);
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("At most this many million points per frame");
    ImGui::SliderFloat("Detail", &lodDetail, 0.05f, 20.0f, "%.2f", ImGuiSliderFlags_Logarithmic);
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Points per pixel of a chunk's size on screen: higher = more points in the distance");
    ImGui::EndDisabled();
    std::size_t drawn = 0, total = 0;
    for (const auto& c : clouds){
        drawn += c.drawnPoints;
        total += (std::size_t)c.numPoints;
    }
    ImGui::Text("Drawn: %.1f M of %.1f M points", drawn / 1e6, total / 1e6);
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
    nfdu8filteritem_t filters[1] = {{"point cloud", "pcd,ply,obj"}};
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

    // Frustum culling and level of detail for all clouds, then one common point budget.
    // While recording, quality matters more than speed: all points of the visible chunks.
    CloudView view;
    view.clipFromObject = camera.cameraMatrix * modelMatrix;
    view.cameraPosition = glm::vec3(glm::inverse(modelMatrix) * glm::vec4(camera.Position, 1.0f));
    view.pixelsPerUnit = (float)framebufferHeight / (2.0f * std::tan(glm::radians(kFovDeg) * 0.5f));
    view.lod = lodEnabled && !path.isRecording();
    for (auto& c : clouds)
        c.prepare(view);
    // Over the point budget: lower the detail for all clouds until it fits. Close chunks
    // stay complete as long as possible, distant ones are thinned out first.
    auto pointsFor = [&](float detail){
        double n = 0.0;
        for (const auto& c : clouds) n += c.pointsForDetail(detail);
        return n;
    };
    float detail = lodDetail;
    double budget = pointBudgetMillions * 1e6;
    if (view.lod && pointsFor(detail) > budget){
        float lo = 1e-6f, hi = detail;   // pointsFor is monotonic: bisect on a log scale
        for (int i = 0; i < 24; i++){
            float mid = std::sqrt(lo * hi);
            (pointsFor(mid) > budget ? hi : lo) = mid;
        }
        detail = lo;
    }
    // thinned out below the chosen detail: larger points close the gaps (not cube/sphere, sized in m)
    float sizeFactor = std::clamp(std::sqrt(lodDetail / detail), 1.0f, 3.0f);

    for (auto& c : clouds){
        PointShader& ps = pointShaders[c.shaderType];
        ps.shader.Activate();
        camera.Matrix(ps.shader, "camMatrix");   // camera matrix + common uniforms (near, far, ...)
        glUniformMatrix4fv(ps.shader.uniform("model"), 1, GL_FALSE, glm::value_ptr(modelMatrix));
        ps.setUniforms(ps.shader, c, settings, ctx);

        if (ps.programPointSize)
            glEnable(GL_PROGRAM_POINT_SIZE);
        float size = ps.programPointSize ? 0.0f : c.pointssize;
        c.Draw(detail, view.lod ? lodDetail : detail, size, size * sizeFactor);
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
