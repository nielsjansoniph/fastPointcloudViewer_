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
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "shaderClass.h" 
#include "Camera.h"
#include "Cloud.h"

#include "stb_image.h"
#include "Texture.h"

#include "Mesh.h"
#include<filesystem>
namespace fs = std::filesystem;


#include <happly.h>
#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
//#include <pcl/io/ply_io.h>
//
//#include <pcl/point_types.h>
#include <nfd.h>


#include <chrono>
#include <cstddef>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>



const unsigned int width = 1600;
const unsigned int height = 900;
 

static void glfw_error_callback(int error, const char* description)
{
    fprintf(stderr, "GLFW Error %d: %s\n", error, description);
}

// Main code
int main(int argc, char * argv[])
{   




    //std::cout << "Input args: " << std::endl;
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

    NFD_Init();

   

    std::cout << "Compiling defaultShader" << std::endl;
    Shader defaultShader("../../default.vert", "../../default.frag");
    std::cout << "Compiling monoColorShader" << std::endl;
    Shader monoColorShader("../../default.vert", "../../monoColor.frag");
    std::cout << "Compiling rgbSphereShader" << std::endl;
    Shader rgbSphereShader("../../default.vert", "../../rgbSphere.frag");
    std::cout << "Compiling cubeShader" << std::endl;
    Shader cubeShader("../../default.vert", "../../cube.frag");
    std::cout << "Compiling meshShader" << std::endl;
    Shader meshShader("../../mesh.vert", "../../mesh.frag");
    

    std::cout << "Compiling colorGradient" << std::endl;
    Shader colorGradientShader("../../default.vert", "../../colorgradient.frag");
    std::cout << "Compiling debugShader" << std::endl;
    Shader debugShader("../../default.vert", "../../debug.frag");
    std::cout << "Compiling zmapShader" << std::endl;
    Shader zmapShader("../../default.vert", "../../zmap.frag");
    std::vector<Cloud> clouds;
 


    /*clouds.push_back(Cloud("scans.ply"));
    clouds[clouds.size()-1].currentShader = &monoColorShader;
    clouds[clouds.size()-1].shaderType = 1;*/

/*
    Assimp::Importer importer;
    const aiScene* scene  = importer.ReadFile("C:/Users/n.janson/OneDrive - IPH Hannover gGmbH/Sonstiges/PasteuralleEG.stl", 
        aiProcess_RemoveComponent       |
        aiProcess_GenNormals            |
        aiProcess_Triangulate           |
        aiProcess_JoinIdenticalVertices |
        aiProcess_SortByPType);

    if (scene == nullptr){
        std::cout << importer.GetErrorString() << std::endl;
    }


    aiMesh* mesh = scene->mMeshes[0];
    std::vector <VertexPosNorCol> v;

    for (unsigned int i = 0; i < mesh->mNumVertices; i++){
        aiVector3f ptmp = mesh->mVertices[i];
        glm::vec3 position{ptmp.y/10.0f, ptmp.z/10.0f, ptmp.x/10.0f};
        //aiColor4D colortmp = mesh->mColors[0][i];
        //glm::vec3 color{colortmp.r, colortmp.g, colortmp.b};
        aiVector3f normals = mesh->mNormals[i];
        glm::vec3 color{(normals.x+1.0f)/2.0f, (normals.y+1.0f)/2.0f, (normals.z+1.0f)/2.0f};
        //glm::vec3 color{1.0f, 1.0f, 1.0f};
        glm::vec2 texCoord{0.0f, 0.0f};

        v.push_back(VertexPosNorCol{position, color, texCoord});
    }

    std::vector <GLuint> f;
    for (unsigned int i = 0; i < mesh->mNumFaces; i++){
        aiFace ftmp = mesh->mFaces[i];
        for (unsigned int ii = 0; ii < ftmp.mNumIndices; ii++){
            f.push_back(ftmp.mIndices[ii]);
        }
    }


    Mesh pyramid(v, f);*/
    
    glEnable(GL_DEPTH_TEST);

    Camera camera(width, height, glm::vec3(0.0f, 0.0f, 2.0f));

    float pointSize = 5;
    float cFactor = 0.5;

    float startDist = 2;
    float endDist = 100;
    float startBrightness = 0.8;
    float endBrightness = 0.2;
    float split = width / 2;
    int res = 1080;
    glm::vec3 startColor = glm::vec3(0.5, 1, 0.5);
    glm::vec3 endColor = glm::vec3(0.0, 0.0, 0.4);

    cv::VideoWriter writer;

    // Main loop
    while (!glfwWindowShouldClose(window))
    {
        static float f = 0.4f;
        static bool useDepthOnSize = true;
        static bool loadButtonPressed = false;
        
        static char filepath[1024];
        static bool replayButtonPressed = false;
        static std::vector <glm::vec3> posPath;
        static int currentReplayPosition = -1;
        int i=0;
        static unsigned int nscreenshots = 0;
        static GLubyte *pixels = nullptr;
        static const GLenum FORMAT = GL_RGBA;
        static const GLuint FORMAT_NBYTES = 4;
        cv::Mat frame;

        glfwPollEvents();
 
        //Imgui Stuff
        {
        // Start the Dear ImGui frame
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        // 1. Show the big demo window (Most of the sample code is in ImGui::ShowDemoWindow()! You can browse its code to learn more about Dear ImGui!).
        //ImGui::ShowDemoWindow();

        // Create a window called "Hello, world!" and append into it.
        ImGui::Begin("Viewer settings");                     
        ImGui::SliderFloat("Near", &camera.nearDist, 0.0f, 1.0f);
        ImGui::SliderFloat("Far", &camera.farDist, 0.0f, 100.0f);
        ImGui::SliderFloat("Speed", &camera.speed, 0.0f, 3.0f);
        ImGui::Text("Background color");
        ImGui::ColorEdit3("", (float*)&clear_color); // Edit 3 floats representing a color
        //ImGui::Checkbox("Use depth for pointsize", &camera.useDepthOnPointsize);
        ImGui::Checkbox("Use depth for point brightness", &camera.useDepthOnPointBrightness);
        ImGui::Checkbox("Use shadow", &camera.useShadow);
        loadButtonPressed = ImGui::Button("Load");
        //show options for each cloud
        for (auto & c : clouds){
            i++;
            ImGui::PushID(i);
            ImGui::Text(c.filename.c_str());
            ImGui::SliderFloat("PointSize", &c.pointssize, 1, 20);
            ImGui::RadioButton("Default shader", &c.shaderType, 0); ImGui::SameLine();
            ImGui::RadioButton("Monocolor shader", &c.shaderType, 1);
            

            //ImGui::RadioButton("RGB sphere shader", &c.shaderType, 2); ImGui::SameLine();
            //ImGui::RadioButton("Cube shader", &c.shaderType, 3); 
            ImGui::RadioButton("ColorGradient shader", &c.shaderType, 4);
            ImGui::RadioButton("debug shader", &c.shaderType, 5);
            ImGui::RadioButton("zMap Shader", &c.shaderType, 6);
            //Show settings depending on shadertype
            switch (c.shaderType){
                case 0:{
                    ImGui::SliderFloat("CFactor", &cFactor, 0.0f, 10.0f);
                    c.currentShader = &defaultShader;
                    break;
                } 
                case 1:{
                    ImGui::ColorEdit3("Point color", (float*)&c.point_color);
                    c.currentShader = &monoColorShader;
                    break;
                }
                case 2:{
                    
                    //c.currentShader = &rgbSphereShader;
                    break;
                }
                case 3:{
                    //c.currentShader = &cubeShader;
                    break;
                }
                case 4:{
                    ImGui::SliderFloat("Start distance", &startDist, 0.0f, 50.f);
                    ImGui::SliderFloat("End distance", &endDist, 0.0f, 500.0f);
                    ImGui::SliderFloat("Start Brightness", &startBrightness, 0.0f, 1.0f);
                    ImGui::SliderFloat("End Brightness", &endBrightness, 0.0f, 1.0f);
                    ImGui::SliderFloat("CFactor", &cFactor, 0.0f, 10.0f);
                    //ImGui::ColorEdit3("Color", (float*)&c.point_color);
                    c.currentShader = &colorGradientShader;
                    break;

                }
                case 5:{
                    ImGui::SliderFloat("Split", &split, 0.0f, (float)width);
                    if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled)){
                        ImGui::SetTooltip("Slide to change debug view border");
                    }
                    ImGui::SliderFloat("Start distance", &startDist, 0.0f, 20.0f);
                    ImGui::SliderFloat("End distance", &endDist, 0.0f, 50.0f);
                    ImGui::ColorEdit3("Start color", (float*)&startColor);
                    ImGui::ColorEdit3("End color", (float*)&endColor);
                    
                    c.currentShader = &debugShader;
                    break;
                }
                case 6:{
                    ImGui::SliderFloat("Split", &split, 0.0f, (float)width);
                    ImGui::SliderFloat("Start distance", &startDist, c.min.position.y, c.max.position.y);
                    ImGui::SliderFloat("End distance", &endDist, c.min.position.y, c.max.position.y);
                    
                    c.currentShader = &zmapShader;
                    break;
                }
            }

            ImGui::NewLine();
            ImGui::PopID();
        }
            
        //ImGui::Text("%d Points", cloud.vertices.size());
        ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / io.Framerate, io.Framerate);
        ImGui::Text("%d Positions stored", posPath.size());
        ImGui::SameLine();
        replayButtonPressed = ImGui::Button("Replay");
        
        ImGui::End();
        // Rendering
        ImGui::Render();
        }


        //add new cloud TODO: catch invalid clouds
        if (loadButtonPressed){
                
            nfdu8char_t *outPath;
            nfdu8filteritem_t filters[1] = { { "point cloud", "pcd,ply" }};
            nfdopendialogu8args_t args = {0};
            args.filterList = filters;
            args.filterCount = 1;
            nfdresult_t result = NFD_OpenDialogU8_With(&outPath, &args);
            if (result == NFD_OKAY)
            {
                puts("File selected");
                puts(outPath);
                clouds.push_back(Cloud(outPath));
                clouds[clouds.size()-1].currentShader = &monoColorShader;
                clouds[clouds.size()-1].shaderType = 1;
                NFD_FreePathU8(outPath);
                float startDist = clouds[clouds.size()-1].min.position.z;
                float endDist = clouds[clouds.size()-1].max.position.z;
            }
            else if (result == NFD_CANCEL)
            {
                puts("User pressed cancel.");
            }
            else 
            {
                printf("Error: %s\n", NFD_GetError());
            }

            //   clouds.push_back(Cloud(filepath));
            // clouds[clouds.size()-1].currentShader = &monoColorShader;
            //  clouds[clouds.size()-1].shaderType = 1;
                loadButtonPressed = false;
        }

        //update window size
        int display_w, display_h;
        glfwGetFramebufferSize(window, &display_w, &display_h);
        //clear window
        glViewport(0, 0, display_w, display_h);
        glClearColor(clear_color.x * clear_color.w, clear_color.y * clear_color.w, clear_color.z * clear_color.w, clear_color.w);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        //set pointsize
        
        //camera rotation and movement update
        //process keypresses only if the curser isn't over the imgui menus
        if (!ImGui::GetIO().WantCaptureMouse){

        //Keyboard movement handling
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
            if (ImGui::IsKeyDown((ImGuiKey)GLFW_KEY_C)){// && posPath.back() != camera.Position){
                if (posPath.size() == 0){
                    posPath.push_back(camera.Position);
                }
                else{
                    glm::vec3 lastPos = posPath.back();
                    if (lastPos.x != camera.Position.x && 
                        lastPos.y != camera.Position.y &&
                        lastPos.z != camera.Position.z){
                        posPath.push_back(camera.Position);
                    }
                }
            }

            camera.Inputs(window);
            camera.updateMatrix(45.0f, 0.1f, 500.0f);
        }

        if (replayButtonPressed){
            replayButtonPressed = false;
            currentReplayPosition = 0;
            pixels = (GLubyte*) malloc(FORMAT_NBYTES * width * height);
            
            int codec = cv::VideoWriter::fourcc('M', 'J', 'P', 'G');
            double fps = 25.0;
            std::string filename = "./asdf.avi";
            cv::Size sz;
            sz.width = width;
            sz.height = height;
            writer.open(filename, codec, fps, sz, true);
            if (!writer.isOpened()) {
                std::cerr << "Could not open the output video file for write\n";
                return -1;
            }
            

        }

        if (currentReplayPosition != -1){
            camera.Position = posPath[currentReplayPosition];
            camera.Inputs(window);
            camera.updateMatrix(45.0f, 0.1f, 500.0f);
            if (currentReplayPosition == posPath.size()-1){
                currentReplayPosition = -1;
                writer.release();
            }
            else{
                currentReplayPosition++;
            }
        }



        //pyramid.Draw(meshShader, camera);

        /*
        meshShader.Activate();

        camera.Matrix(meshShader, "camMatrix");

        //glUniform1f(uniID, 0.5f);
        //popCat.Bind();

        VAO.Bind();

		// Draw primitives, number of indices, datatype of indices, index of indices
		glDrawElements(GL_TRIANGLES, sizeof(indices)/sizeof(int), GL_UNSIGNED_INT, 0);
        */

        for (auto & c : clouds){
            glPointSize(c.pointssize);

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
                    glUniform3fv(id, 1, glm::value_ptr(c.point_color));
                    break;
                }
                case 4:{
                    GLuint id = glGetUniformLocation(colorGradientShader.ID, "cFactor");
                    glUniform1f(id, cFactor);
                    id = glGetUniformLocation(colorGradientShader.ID, "startDist");
                    glUniform1f(id, startDist);
                    id = glGetUniformLocation(colorGradientShader.ID, "endDist");
                    glUniform1f(id, endDist);
                    break;
                }
                case 5:{
                    GLuint id = glGetUniformLocation(debugShader.ID, "startDist");
                    glUniform1f(id, startDist);
                    id = glGetUniformLocation(debugShader.ID, "endDist");
                    glUniform1f(id, endDist);
                    id = glGetUniformLocation(debugShader.ID, "split");
                    glUniform1f(id, split);
                    id = glGetUniformLocation(debugShader.ID, "startColor");
                    glUniform3fv(id, 1, glm::value_ptr(startColor));
                    id = glGetUniformLocation(debugShader.ID, "endColor");
                    glUniform3fv(id, 1, glm::value_ptr(endColor));
                    break;
                }
                case 6:{
                    GLuint id = glGetUniformLocation(zmapShader.ID, "startDist");
                    glUniform1f(id, startDist);
                    id = glGetUniformLocation(zmapShader.ID, "endDist");
                    glUniform1f(id, endDist);
                    id = glGetUniformLocation(zmapShader.ID, "split");
                    glUniform1f(id, split);
                    break;
                }
            }
        
            c.Draw(camera);
        }



        

        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        glfwSwapBuffers(window);

        if (currentReplayPosition != -1){
            //glReadPixels(0, 0, width, height, FORMAT, GL_UNSIGNED_BYTE, pixels);
            frame.create(height, width, CV_8UC3);

            glPixelStorei(GL_PACK_ALIGNMENT, (frame.step & 3) ? 1 : 4);
            glPixelStorei(GL_PACK_ROW_LENGTH, frame.step/frame.elemSize());
            glReadPixels(0, 0, frame.cols, frame.rows, GL_BGR, GL_UNSIGNED_BYTE, frame.data);
            //cv::flip(frame, true, 0);
            writer.write(frame);
            42;


        }


    }



    // Cleanup
    //defaultShader.Delete();
    //monoColorShader.Delete();
    //rgbSphereShader.Delete();
    //cubeShader.Delete();  
    NFD_Quit();


    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    glfwDestroyWindow(window);
    glfwTerminate();

    return 0;
}
