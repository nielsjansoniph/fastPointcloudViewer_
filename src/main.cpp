// Fast first person point cloud viewer – entry point: command line, window and ImGui setup, main loop.
// The viewer itself is in Viewer.cpp.
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <string>
#include <vector>

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <nfd.h>

#include "Viewer.h"

#ifdef _WIN32
// Laptops with two GPUs: ask the NVIDIA / AMD driver for the dedicated GPU
extern "C" {
__declspec(dllexport) unsigned long NvOptimusEnablement = 1;
__declspec(dllexport) int AmdPowerXpressRequestHighPerformance = 1;
}
#endif

// Linux laptops with NVIDIA PRIME render on the integrated GPU unless asked otherwise
// (102 M points: 2.4 fps on Intel UHD vs. 35 fps on an RTX 4080). Must run before glfwInit,
// which loads the OpenGL library. Skipped if no NVIDIA driver is loaded, if the user chose
// a vendor already, or with FPCV_INTEGRATED_GPU=1.
static void preferDedicatedGpu()
{
#ifndef _WIN32
    if (std::getenv("FPCV_INTEGRATED_GPU") || std::getenv("__GLX_VENDOR_LIBRARY_NAME"))
        return;
    std::error_code ec;
    if (!std::filesystem::exists("/proc/driver/nvidia/version", ec))
        return;
    setenv("__NV_PRIME_RENDER_OFFLOAD", "1", 0);
    setenv("__GLX_VENDOR_LIBRARY_NAME", "nvidia", 0);
#endif
}

static void glfwErrorCallback(int error, const char* description)
{
    std::cerr << "GLFW Error " << error << ": " << description << std::endl;
}

int main(int argc, char* argv[])
{
    // Command line: point cloud files to load at startup
    std::vector<std::string> startupFiles;
    for (int a = 1; a < argc; a++){
        std::string arg = argv[a];
        if (arg == "-h" || arg == "--help"){
            std::cout << "Usage: " << argv[0] << " [cloud.ply|cloud.pcd|cloud.obj ...]\n"
                      << "Loads the given point clouds (.ply, .pcd, or the vertices of an .obj) at startup.\n";
            return 0;
        }
        startupFiles.push_back(arg);
    }

    preferDedicatedGpu();
    glfwSetErrorCallback(glfwErrorCallback);
    if (!glfwInit())
        return 1;
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    GLFWwindow* window = glfwCreateWindow(1600, 900, "Fast first person point cloud viewer", nullptr, nullptr);
    if (window == nullptr){
        std::cerr << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        return 1;
    }
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);   // vsync
    gladLoadGL();
    std::cout << "OpenGL renderer: " << (const char*)glGetString(GL_RENDERER) << std::endl;

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGui::GetIO().ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard | ImGuiConfigFlags_NavEnableGamepad;
    ImGui::StyleColorsDark();
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 330");

    NFD_Init();

    {
        Viewer viewer(window);
        for (const auto& path : startupFiles)
            viewer.loadCloud(path);

        while (!glfwWindowShouldClose(window)){
            glfwPollEvents();
            viewer.frame();
            viewer.afterRender();   // video capture must happen before the swap
            glfwSwapBuffers(window);
        }
    }

    NFD_Quit();
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    glfwDestroyWindow(window);
    glfwTerminate();
    return 0;
}
