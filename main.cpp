// Dear ImGui: standalone example application for GLFW + OpenGL 3, using programmable pipeline
// (GLFW is a cross-platform general purpose library for handling windows, inputs, OpenGL/Vulkan/Metal graphics context creation, etc.)

// Learn about Dear ImGui:
// - FAQ                  https://dearimgui.com/faq
// - Getting Started      https://dearimgui.com/getting-started
// - Documentation        https://dearimgui.com/docs (same as your local docs/ folder)
// - Introduction, links and more at the top of imgui.cpp

#define GLEW_BUILD
#define GLFW_DLL

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include <stdio.h>
#define GL_SILENCE_DEPRECATION
#include <GLEW/glew.h>
#include <GLFW/glfw3.h> // Will drag system OpenGL headers
#include <math.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <algorithm>

#include "shaderClass.h" 
#include "VAO.h"
#include "VBO.h"
#include "EBO.h"
#include "Camera.h"

#include "happly.h"


const unsigned int width = 1600;
const unsigned int height = 900;


GLfloat vertices[] =
{ //     COORDINATES     /        COLORS      /
	-0.5f, 0.0f,  0.5f,     0.83f, 0.70f, 0.44f,
	-0.5f, 0.0f, -0.5f,     0.83f, 0.70f, 0.44f,
	 0.5f, 0.0f, -0.5f,     0.83f, 0.70f, 0.44f,
	 0.5f, 0.0f,  0.5f,     0.83f, 0.70f, 0.44f,
	 0.0f, 0.8f,  0.0f,     0.92f, 0.86f, 0.76f,
};

// Indices for vertices order
GLuint indices[] =
{
	0, 1, 2,
	0, 2, 3,
	0, 1, 4,
	1, 2, 4,
	2, 3, 4,
	3, 0, 4
};

static void glfw_error_callback(int error, const char* description)
{
    fprintf(stderr, "GLFW Error %d: %s\n", error, description);
}

// Main code
int main(int, char**)
{


    //happly::PLYData plyIn("C:/Users/n.janson/OneDrive - IPH Hannover gGmbH/Punktwolken/jurgensen_merged_cleaned_Verts.ply");
    happly::PLYData plyIn("scan_v1+2.ply");
    //happly::PLYData plyIn("scans.ply");


    std::vector<float> coordx = plyIn.getElement("vertex").getProperty<float>("x");
    std::vector<float> coordy = plyIn.getElement("vertex").getProperty<float>("y");
    std::vector<float> coordz = plyIn.getElement("vertex").getProperty<float>("z");

    const auto [minX, maxX] = std::minmax_element(begin(coordx), end(coordx));
    const auto [minY, maxY] = std::minmax_element(begin(coordy), end(coordy));
    const auto [minZ, maxZ] = std::minmax_element(begin(coordz), end(coordz));

    const auto min = std::min({minX, minY, minZ});
    const auto max = std::min({maxX, maxY, maxZ});

    auto range = max - min;

    int n = coordx.size();

    //GLfloat vertices[n*6] = {0.5f};

    GLfloat *vertices = (GLfloat *) malloc(n*6*sizeof(GLfloat));

    for (int i=0; i<n; i++){
        vertices[i*6+0] = coordx[i];
        vertices[i*6+1] = coordy[i];
        vertices[i*6+2] = coordz[i];
        vertices[i*6+3] = 1.0f;
        vertices[i*6+4] = 1.0f;
        vertices[i*6+5] = 1.0f;
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
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); (void)io;
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;     // Enable Keyboard Controls
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;      // Enable Gamepad Controls

    
    

    // Setup Dear ImGui style
    ImGui::StyleColorsDark();
    //ImGui::StyleColorsLight();

    // Setup Platform/Renderer backends
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init(glsl_version);

    // Our state
    bool show_demo_window = true;
    bool show_another_window = false;
    ImVec4 clear_color = ImVec4(0.0f, 0.0f, 0.0f, 1.00f);

     glfwMakeContextCurrent(window);
    if (glewInit()) {
        printf("failed to initialize OpenGL\n");
        return -1;
    } 


    Shader shaderProgram("default.vert", "default.frag");
    VAO VAO1;
    VAO1.Bind();

    VBO VBO1(vertices, n*6*sizeof(GLfloat));
    EBO EBO1(indices, sizeof(indices));

    VAO1.LinkAttrib(VBO1, 0, 3, GL_FLOAT, 6 * sizeof(float), (void*)0);
    VAO1.LinkAttrib(VBO1, 1, 3, GL_FLOAT, 6 * sizeof(float), (void*)(3 * sizeof(float)));

    VAO1.Unbind();
    VBO1.Unbind();
    EBO1.Unbind();


    glEnable(GL_DEPTH_TEST);

    Camera camera(width, height, glm::vec3(0.0f, 0.0f, 2.0f));
    int direction = 0;

    int keyDelayCounter = 0;

    GLuint nearUniID = glGetUniformLocation(shaderProgram.ID, "near"); 
    GLuint farUniID = glGetUniformLocation(shaderProgram.ID, "far"); 

    float near = 0.1f;
    float far = 20.0f;
    int pointSize = 5;

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
       // if (show_demo_window)
        //    ImGui::ShowDemoWindow(&show_demo_window);


        static float f = 0.4f;
        // 2. Show a simple window that we create ourselves. We use a Begin/End pair to create a named window.
        {
            
            static int counter = 0;

            ImGui::Begin("Viewer settings");                          // Create a window called "Hello, world!" and append into it.

            ImGui::Text("This is some useful text.");               // Display some text (you can use a format strings too)
            

            ImGui::SliderFloat("Near", &near, 0.0f, 1.0f);            // Edit 1 float using a slider from 0.0f to 1.0f
            ImGui::SliderFloat("Far", &far, 0.0f, 100.0f);
            ImGui::SliderFloat("Speed", &camera.speed, 0.0f, 3.0f);
            ImGui::SliderInt("PointSize", &pointSize, 1, 20);
            ImGui::ColorEdit3("BackgroundColor", (float*)&clear_color); // Edit 3 floats representing a color
              

            if (ImGui::Button("Button")){
                for (int i=0; i<n; i++){
                    GLfloat buf = vertices[i*6+0];
                    vertices[i*6+0] = vertices[i*6+1];
                    vertices[i*6+1] = buf;
                }
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


        shaderProgram.Activate();

        if (ImGui::IsKeyPressed((ImGuiKey)GLFW_KEY_W))
            camera.forward();
        
        if (ImGui::IsKeyPressed((ImGuiKey)GLFW_KEY_A))
            camera.left();

        if (ImGui::IsKeyPressed((ImGuiKey)GLFW_KEY_S))
            camera.backward();

        if (ImGui::IsKeyPressed((ImGuiKey)GLFW_KEY_D))
            camera.right();

        if (ImGui::IsKeyPressed((ImGuiKey)GLFW_KEY_R)){
            camera.up();
        }
        if (ImGui::IsKeyPressed((ImGuiKey)GLFW_KEY_F)){
            camera.down();
        }

        camera.Inputs(window);
        camera.Matrix(45.0f, 0.1f, 100.0f, shaderProgram, "camMatrix");

        glUniform1f(nearUniID, near);
        glUniform1f(farUniID, far);

     
        VAO1.Bind();
        glPointSize(pointSize);
        glDrawArrays(GL_POINTS, 0, n);
        //glDrawArrays(GL_TRIANGLES, 0, 3);
        //glDrawElements(GL_TRIANGLES, sizeof(indices)/sizeof(int), GL_UNSIGNED_INT, 0);
        //glDrawElements(GL_POINTS, sizeof(indices)/sizeof(int), GL_UNSIGNED_INT, 0);
        

        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        glfwSwapBuffers(window);
    }

    // Cleanup
    VAO1.Delete();
    VBO1.Delete();
    EBO1.Delete();
    shaderProgram.Delete();
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    glfwDestroyWindow(window);
    glfwTerminate();

    return 0;
}
