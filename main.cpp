// Dear ImGui: standalone example application for GLFW + OpenGL 3, using programmable pipeline
// (GLFW is a cross-platform general purpose library for handling windows, inputs, OpenGL/Vulkan/Metal graphics context creation, etc.)

// Learn about Dear ImGui:
// - FAQ                  https://dearimgui.com/faq
// - Getting Started      https://dearimgui.com/getting-started
// - Documentation        https://dearimgui.com/docs (same as your local docs/ folder)
// - Introduction, links and more at the top of imgui.cpp


//#include "imgui.h"
#include <iostream>
#include <stdio.h>
//#include <GLEW/glew.h>
#include <glad/glad.h>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <imgui_impl_opengl3_loader.h>

#include <GLFW/glfw3.h> // Will drag system OpenGL headers
#include <math.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <algorithm>
#include "shaderClass.h" 
//#include "EBO.h"
#include "Camera.h"
#include "Cloud.h"

#include <pcl/io/ply_io.h>
//#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>



const unsigned int width = 1600;
const unsigned int height = 900;
 

static void glfw_error_callback(int error, const char* description)
{
    fprintf(stderr, "GLFW Error %d: %s\n", error, description);
}

// Main code
int main(int argc, char * argv[])
{   

    std::cout << "Input args: " << std::endl;
    for (int i=0;i<argc;i++){
       std::cout << i << " : " << argv[i] << std::endl; 
    }

    glfwSetErrorCallback(glfw_error_callback);
    if (!glfwInit())
        return 1;

    // GL 3.0 + GLSL 130
    const char* glsl_version = "#version 130";
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);  // 3.2+ only


    // Create window with graphics context
    GLFWwindow* window = glfwCreateWindow(width, height, "Fast first person point cloud viewer", nullptr, nullptr);
    if (window == nullptr){
		std::cout << "Failed to create GLFW window" << std::endl;
		glfwTerminate();
		return -1;
	}
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1); // Enable vsync

    // Setup Dear ImGui context
    //IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); (void)io;
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;     // Enable Keyboard Controls
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;      // Enable Gamepad Controls

    
    

    // Setup Dear ImGui style
    ImGui::StyleColorsDark();

    // Setup Platform/Renderer backends
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init(glsl_version);

    ImVec4 clear_color = ImVec4(0.0f, 0.0f, 0.0f, 1.00f);
    glm::vec3 point_color = glm::vec3(1.0f);
    

    glfwMakeContextCurrent(window);

    gladLoadGL();


    Shader defaultShader("../../default.vert", "../../default.frag");
    Shader monoColorShader("../../default.vert", "../../monoColor.frag");
    Shader rgbSphereShader("../../default.vert", "../../rgbSphere.frag");
    Shader cubeShader("../../default.vert", "../../cube.frag");


    //Cloud cloud(vertices);

    std::vector<Cloud> clouds;

    Cloud cloud("scans.ply");
    cloud.currentShader = &defaultShader;
    cloud.shaderType = 0;
    clouds.push_back(cloud);

    clouds[0].shaderType = 1;

    


    glEnable(GL_DEPTH_TEST);

    Camera camera(width, height, glm::vec3(0.0f, 0.0f, 2.0f));

    float pointSize = 5;
    float cFactor = 1.0;

    // Main loop
    while (!glfwWindowShouldClose(window))
    {

        glfwPollEvents();

        // Start the Dear ImGui frame
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        // 1. Show the big demo window (Most of the sample code is in ImGui::ShowDemoWindow()! You can browse its code to learn more about Dear ImGui!).
        //ImGui::ShowDemoWindow();


        static float f = 0.4f;
        static bool useDepthOnSize = true;
    

        ImGui::Begin("Viewer settings");                          // Create a window called "Hello, world!" and append into it.

        ImGui::SliderFloat("Near", &camera.near, 0.0f, 1.0f);            // Edit 1 float using a slider from 0.0f to 1.0f
        ImGui::SliderFloat("Far", &camera.far, 0.0f, 100.0f);
        ImGui::SliderFloat("Speed", &camera.speed, 0.0f, 3.0f);
        ImGui::SliderFloat("PointSize", &pointSize, 1, 20);
        ImGui::ColorEdit3("BackgroundCo tlor", (float*)&clear_color); // Edit 3 floats representing a color
            
        ImGui::Checkbox("Use depth for pointsize", &camera.useDepthOnPointsize);
        ImGui::Checkbox("Use depth for point brightness", &camera.useDepthOnPointBrightness);
        ImGui::Checkbox("Use shadow", &camera.useShadow);
        

        //for (int i=0; i<clouds.size(); i++){
        for (auto & c : clouds){
            ImGui::RadioButton("Default shader", &c.shaderType, 0); ImGui::SameLine();
            ImGui::RadioButton("Monocolor shader", &c.shaderType, 1);
            ImGui::RadioButton("RGB sphere shader", &c.shaderType, 2); ImGui::SameLine();
            ImGui::RadioButton("Cube shader", &c.shaderType, 3); 

            //Show settings depending on shadertype
            switch (c.shaderType){
                case 0:{
                    ImGui::SliderFloat("CFactor", &cFactor, 0.0f, 10.0f);
                    c.currentShader = &defaultShader;
                    break;
                } 
                case 1:{
                    ImGui::ColorEdit3("Point color", (float*)&point_color);
                    c.currentShader = &monoColorShader;
                    break;
                }
                case 2:{
                    c.currentShader = &rgbSphereShader;
                    break;
                }
                case 3:{
                    c.currentShader = &cubeShader;
                    break;
                }
            }
        }
            
        //ImGui::Text("%d Points", cloud.vertices.size());
        ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / io.Framerate, io.Framerate);
        
        ImGui::End();
        // Rendering
        ImGui::Render();

        int display_w, display_h;
        glfwGetFramebufferSize(window, &display_w, &display_h);
        glViewport(0, 0, display_w, display_h);
        glClearColor(clear_color.x * clear_color.w, clear_color.y * clear_color.w, clear_color.z * clear_color.w, clear_color.w);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glPointSize(pointSize);

        
        //Keyboard movement handling
        {
            if (ImGui::IsKeyDown((ImGuiKey)GLFW_KEY_W))
                camera.forward();
            
            if (ImGui::IsKeyDown((ImGuiKey)GLFW_KEY_A))
                camera.left();

            if (ImGui::IsKeyDown((ImGuiKey)GLFW_KEY_S))
                camera.backward();

            if (ImGui::IsKeyDown((ImGuiKey)GLFW_KEY_D))
                camera.right();

            if (ImGui::IsKeyDown((ImGuiKey)GLFW_KEY_R)){
                camera.up();
            }
            if (ImGui::IsKeyDown((ImGuiKey)GLFW_KEY_F)){
                camera.down();
            }
        }


        if (!ImGui::GetIO().WantCaptureMouse)
            camera.Inputs(window);
        camera.updateMatrix(45.0f, 0.1f, 200.0f);


        //for (int i=0; i<clouds.size(); i++){
        for (auto & c : clouds){

            c.currentShader->Activate();

            camera.Matrix(*c.currentShader, "camMatrix");

            switch (c.shaderType){
                case 0:{
                    GLuint id = glGetUniformLocation(defaultShader.ID, "cFactor");
                    glUniform1f(id, cFactor);
                    break;
                }
                case 1:{
                    GLuint id = glGetUniformLocation(monoColorShader.ID, "color");
                    glUniform3fv(id, 1, glm::value_ptr(point_color));
                    break;
                }
            }
        
            c.Draw(camera);
        }


        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        glfwSwapBuffers(window);
    }



    // Cleanup
    defaultShader.Delete();
    monoColorShader.Delete();
    rgbSphereShader.Delete();
    cubeShader.Delete();

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    glfwDestroyWindow(window);
    glfwTerminate();

    return 0;
}
