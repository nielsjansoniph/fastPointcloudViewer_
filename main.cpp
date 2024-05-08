// Dear ImGui: standalone example application for GLFW + OpenGL 3, using programmable pipeline
// (GLFW is a cross-platform general purpose library for handling windows, inputs, OpenGL/Vulkan/Metal graphics context creation, etc.)

// Learn about Dear ImGui:
// - FAQ                  https://dearimgui.com/faq
// - Getting Started      https://dearimgui.com/getting-started
// - Documentation        https://dearimgui.com/docs (same as your local docs/ folder)
// - Introduction, links and more at the top of imgui.cpp


#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include <stdio.h>
#define GL_SILENCE_DEPRECATION
//#include <GLEW/glew.h>
#include <glad/glad.h>
#include <GLFW/glfw3.h> // Will drag system OpenGL headers
#include <math.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <algorithm>
#include "happly.h"
#include "shaderClass.h" 
//#include "EBO.h"
#include "Camera.h"
#include "Cloud.h"
#include <iostream>


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
    
    //happly::PLYData plyIn(argv[1]);
    happly::PLYData plyIn("C:/Users/n.janson/OneDrive - IPH Hannover gGmbH/fastPointcloudViewer/scans.ply");


    std::vector<float> coordx = plyIn.getElement("vertex").getProperty<float>("x");
    std::vector<float> coordy = plyIn.getElement("vertex").getProperty<float>("y");
    std::vector<float> coordz = plyIn.getElement("vertex").getProperty<float>("z");

    std::pair<std::vector<float>::iterator, std::vector<float>::iterator> mmx = std::minmax_element(begin(coordx), end(coordx));
    std::pair<std::vector<float>::iterator, std::vector<float>::iterator> mmy = std::minmax_element(begin(coordy), end(coordy));
    std::pair<std::vector<float>::iterator, std::vector<float>::iterator> mmz = std::minmax_element(begin(coordz), end(coordz));

    float min = std::min({*mmx.first, *mmy.first, *mmz.first});
    float max = std::max({*mmx.second, *mmy.second, *mmz.second});

    float range = max - min;

    int n = static_cast<int>(coordx.size());

    std::vector<Vertex> vertices;
    Vertex vertex;
    for (int i=0; i<n; i++){
        vertex.position[0] = coordx[i];
        vertex.position[1] = coordy[i]; 
        vertex.position[2] = coordz[i];
        vertex.color[0] = 1.0f; //(coordx[i] - *mmx.first) / (*mmx.second - *mmx.first)/2.0f;
        vertex.color[1] = 1.0f; //(coordy[i] - *mmy.first) / (*mmy.second - *mmy.first)/2.0f;
        vertex.color[2] = 1.0f; //(coordz[i] - *mmz.first) / (*mmz.second - *mmz.first)/2.0f;
        vertices.push_back(vertex);
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
    GLFWwindow* window = glfwCreateWindow(width, height, "Dear ImGui GLFW+OpenGL3 example", nullptr, nullptr);
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

    // Our state
    bool show_demo_window = false;
    bool show_another_window = false;
    ImVec4 clear_color = ImVec4(0.0f, 0.0f, 0.0f, 1.00f);
    glm::vec3 point_color = glm::vec3(1.0f);
    

     glfwMakeContextCurrent(window);
   /*if (glewInit()) {
        printf("failed to initialize OpenGL\n");
        return -1;
    }*/
    gladLoadGL();


    Shader defaultShader("../../default.vert", "../../default.frag");
    Shader monoColorShader("../../default.vert", "../../monoColor.frag");
    Shader rgbSphereShader("../../default.vert", "../../rgbSphere.frag");
    Shader cubeShader("../../default.vert", "../../cube.frag");

    Cloud cloud(vertices);



    glEnable(GL_DEPTH_TEST);

    Camera camera(width, height, glm::vec3(0.0f, 0.0f, 2.0f));
    int direction = 0;

    int keyDelayCounter = 0;

    float pointSize = 5;

    int shaderSelection = 0;
    float cFactor = 1.0;

    // Main loop
    while (!glfwWindowShouldClose(window))
    {
        // Poll and handle events (inputs, window resize, etc.)
        // You can read the io.WantCaptureMouse, io.WantCaptureKeyboard flags to tell if dear imgui wants to use your inputs.
        // - When io.WantCaptureMouse is true, do not dispatch mouse input data to your main application, or clear/overwrite your copy of the mouse data.
        // - When io.WantCaptureKeyboard is true, do not dispatch keyboard input data to your main application, or clear/overwrite your copy of the keyboard data.
        // Generally you may always pass all inputs to dear imgui, and hide them from your application based on those two flags.
        glfwPollEvents();

        // Start the Dear ImGui frame
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        // 1. Show the big demo window (Most of the sample code is in ImGui::ShowDemoWindow()! You can browse its code to learn more about Dear ImGui!).
        if (show_demo_window)
            ImGui::ShowDemoWindow(&show_demo_window);


        static float f = 0.4f;
        static bool useDepthOnSize = true;
        
        // 2. Show a simple window that we create ourselves. We use a Begin/End pair to create a named window.
        {
            
            static int counter = 0;

            ImGui::Begin("Viewer settings");                          // Create a window called "Hello, world!" and append into it.

            if (ImGui::GetIO().WantCaptureMouse)
                ImGui::Text("Focused");      
            else   
                ImGui::Text("Not focused");  
            

            ImGui::SliderFloat("Near", &camera.near, 0.0f, 1.0f);            // Edit 1 float using a slider from 0.0f to 1.0f
            ImGui::SliderFloat("Far", &camera.far, 0.0f, 100.0f);
            ImGui::SliderFloat("Speed", &camera.speed, 0.0f, 3.0f);
            ImGui::SliderFloat("PointSize", &pointSize, 1, 20);
            ImGui::ColorEdit3("BackgroundCo tlor", (float*)&clear_color); // Edit 3 floats representing a color
              
            ImGui::Checkbox("Use depth for pointsize", &camera.useDepthOnPointsize);
            ImGui::Checkbox("Use depth for point brightness", &camera.useDepthOnPointBrightness);
            ImGui::Checkbox("Use shadow", &camera.useShadow);
            
            ImGui::RadioButton("Default shader", &shaderSelection, 0); ImGui::SameLine();
            ImGui::RadioButton("Monocolor shader", &shaderSelection, 1);
            ImGui::RadioButton("RGB sphere shader", &shaderSelection, 2); ImGui::SameLine();
            ImGui::RadioButton("Cube shader", &shaderSelection, 3); 

            switch (shaderSelection){
                case 0:
                    ImGui::SliderFloat("CFactor", &cFactor, 0.0f, 10.0f);
                break;
                case 1:
                    ImGui::ColorEdit3("Point color", (float*)&point_color);
                break;
            }


            if (ImGui::Button("Button")){
                counter++;
            }                           // Buttons return true when clicked (most widgets return true when edited/activated)
                
            ImGui::SameLine();
            ImGui::Text("%d Points", n);

            ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / io.Framerate, io.Framerate);
            
            ImGui::End();
        }


        // Rendering
        ImGui::Render();
        int display_w, display_h;
        glfwGetFramebufferSize(window, &display_w, &display_h);
        glViewport(0, 0, display_w, display_h);
        glClearColor(clear_color.x * clear_color.w, clear_color.y * clear_color.w, clear_color.z * clear_color.w, clear_color.w);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        switch (shaderSelection){
            case 0:{
                defaultShader.Activate();
                break;
            }
            case 1:{
                monoColorShader.Activate();
                break;
            }
            case 2:{
                rgbSphereShader.Activate();
            }
            case 3:{
                cubeShader.Activate();
            }
        }


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


        if (!ImGui::GetIO().WantCaptureMouse)
            camera.Inputs(window);
        camera.updateMatrix(45.0f, 0.1f, 200.0f);
        camera.Matrix(defaultShader, "camMatrix");


        switch (shaderSelection){
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
     
        glPointSize(pointSize);
        //glDrawArrays(GL_POINTS, 0, n);


        switch (shaderSelection){
            case 0:
                cloud.Draw(defaultShader, camera);
            break;
            case 1:
                cloud.Draw(monoColorShader, camera);
            break;
            case 2:
                cloud.Draw(rgbSphereShader, camera);
            break;
            case 3:
                cloud.Draw(cubeShader, camera);
            break;
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
