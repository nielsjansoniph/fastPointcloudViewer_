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
//#include <imgui_impl_opengl3_loader.h> // internal ImGui header, clashes with glad

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
#include <cmath>
#include <cfloat>
#include <glm/gtc/matrix_transform.hpp>
#include <algorithm>

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>



const unsigned int width = 1600;
const unsigned int height = 900;
 

// Slider with -/+ buttons left and right; each click divides/multiplies the value by `factor`
static bool SliderFloatWithButtons(const char* label, float* v, float vMin, float vMax, float factor = 1.25f, const char* format = "%.3f", ImGuiSliderFlags flags = 0)
{
    ImGui::PushID(label);
    bool changed = false;
    float buttonWidth = ImGui::GetFrameHeight();
    float spacing = ImGui::GetStyle().ItemInnerSpacing.x;
    float clickMin = vMin > 0.0f ? vMin : 0.001f;   // keep clicking from getting stuck at 0
    if (ImGui::Button("-", ImVec2(buttonWidth, 0))){
        *v = std::max(clickMin, *v / factor);
        changed = true;
    }
    ImGui::SameLine(0, spacing);
    ImGui::SetNextItemWidth(ImGui::CalcItemWidth() - 2 * (buttonWidth + spacing));
    changed |= ImGui::SliderFloat("##slider", v, vMin, vMax, format, flags);
    ImGui::SameLine(0, spacing);
    if (ImGui::Button("+", ImVec2(buttonWidth, 0))){
        *v = std::min(vMax, std::max(clickMin, *v) * factor);
        changed = true;
    }
    ImGui::SameLine(0, spacing);
    ImGui::TextUnformatted(label);
    ImGui::PopID();
    return changed;
}

static void glfw_error_callback(int error, const char* description)
{
    fprintf(stderr, "GLFW Error %d: %s\n", error, description);
}

// Main code
int main(int argc, char * argv[])
{   




    // Command line: point cloud files to load at startup
    std::vector<std::string> startupFiles;
    for (int a = 1; a < argc; a++){
        std::string arg = argv[a];
        if (arg == "-h" || arg == "--help"){
            std::cout << "Usage: " << argv[0] << " [cloud.ply|cloud.pcd ...]\n"
                      << "Loads the given point clouds (.ply or .pcd) at startup.\n";
            return 0;
        }
        startupFiles.push_back(arg);
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
    Shader cubeShader("../../cube.vert", "../../cube.frag");
    std::cout << "Compiling meshShader" << std::endl;
    Shader meshShader("../../mesh.vert", "../../mesh.frag");
    

    std::cout << "Compiling colorGradient" << std::endl;
    Shader colorGradientShader("../../default.vert", "../../colorgradient.frag");
    std::cout << "Compiling debugShader" << std::endl;
    Shader debugShader("../../default.vert", "../../debug.frag");
    std::cout << "Compiling zmapShader" << std::endl;
    Shader zmapShader("../../default.vert", "../../zmap.frag");
    std::cout << "Compiling sphereShader" << std::endl;
    Shader sphereShader("../../sphere.vert", "../../sphere.frag");
    std::cout << "Compiling axisShader" << std::endl;
    Shader axisShader("../../axis.vert", "../../axis.frag");
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

    Camera camera(width, height, glm::vec3(0.0f, -2.0f, 0.0f));

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
    cv::Size recordSize(width, height);

    // Loads a point cloud (used by the Load button and the command line)
    auto loadCloud = [&](const std::string& path){
        std::string ext = std::filesystem::path(path).extension().string();
        std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
        if (!std::filesystem::is_regular_file(path)){
            std::cerr << "File not found: " << path << std::endl;
            return;
        }
        if (ext != ".ply" && ext != ".pcd"){
            std::cerr << "Unsupported file type (only .ply and .pcd): " << path << std::endl;
            return;
        }
        std::cout << "Loading " << path << std::endl;
        try {
            clouds.push_back(Cloud(path));
        }
        catch (const std::exception& e) {
            std::cerr << "Error loading point cloud: " << e.what() << std::endl;
            return;
        }
        clouds.back().currentShader = &monoColorShader;
        clouds.back().shaderType = 1;
        std::cout << "Loaded " << clouds.back().numPoints << " points" << std::endl;
    };
    for (const auto& path : startupFiles)
        loadCloud(path);

    // Model orientation: rotated in 90 degree steps around the center of the loaded clouds
    glm::mat4 modelRotation = glm::mat4(1.0f);
    glm::mat4 modelMatrix = glm::mat4(1.0f);
    glm::vec3 modelCenter = glm::vec3(0.0f);

    // Coordinate axes (unit length, scaled by axisLength), drawn at modelCenter
    bool showAxes = true;
    float axisLength = 1.0f;
    std::vector<VertexPosCol> axisVertices = {
        {{0.0f, 0.0f, 0.0f}, {1.0f, 0.2f, 0.2f}}, {{1.0f, 0.0f, 0.0f}, {1.0f, 0.2f, 0.2f}},
        {{0.0f, 0.0f, 0.0f}, {0.2f, 1.0f, 0.2f}}, {{0.0f, 1.0f, 0.0f}, {0.2f, 1.0f, 0.2f}},
        {{0.0f, 0.0f, 0.0f}, {0.3f, 0.5f, 1.0f}}, {{0.0f, 0.0f, 1.0f}, {0.3f, 0.5f, 1.0f}},
    };
    VAO axisVAO;
    axisVAO.Bind();
    VBO axisVBO(axisVertices);
    axisVAO.LinkAttrib(axisVBO, 0, 3, GL_FLOAT, sizeof(VertexPosCol), (void*)0);
    axisVAO.LinkAttrib(axisVBO, 1, 3, GL_FLOAT, sizeof(VertexPosCol), (void*)(3 * sizeof(float)));
    axisVAO.Unbind();
    axisVBO.Unbind();

    // Main loop
    while (!glfwWindowShouldClose(window))
    {
        static float f = 0.4f;
        static bool useDepthOnSize = true;
        static bool loadButtonPressed = false;
        
        static char filepath[1024];
        static bool replayButtonPressed = false;
        static bool replayNextButtonPressed = false;
        static bool replayToEndButtonPressed = false;
        static std::vector <glm::vec3> posPath;
        static std::vector <glm::vec3> posPathInterpolated;
        static std::vector <glm::vec3> oriPath;
        static std::vector <glm::vec3> oriPathInterpolated;
        static int posPathSelection = -1;
        static int currentReplayPosition = -1;
        static int wayPointToDelete = -1;
        static int wayPointUp = -1;
        static int wayPointDown = -1;

        int i=0;
        static unsigned int nscreenshots = 0;
        static const GLenum FORMAT = GL_RGBA;
        static const GLuint FORMAT_NBYTES = 4;
        static bool loop = true;
        static bool record = false;
        cv::Mat frame;

        glfwPollEvents();
 
        //Imgui Stuff
        {
        // Start the Dear ImGui frame
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        // Create a window called "Hello, world!" and append into it.
        ImGui::Begin("Viewer settings");                     
        SliderFloatWithButtons("Near", &camera.nearDist, 0.0f, 1.0f);
        SliderFloatWithButtons("Far", &camera.farDist, 0.0f, 100.0f);
        SliderFloatWithButtons("Speed (m/s)", &camera.speed, 0.0f, 50.0f, 1.25f, "%.2f", ImGuiSliderFlags_Logarithmic);
        ImGui::Text("Background color");
        ImGui::ColorEdit3("", (float*)&clear_color); // Edit 3 floats representing a color
        //ImGui::Checkbox("Use depth for pointsize", &camera.useDepthOnPointsize);
        ImGui::Checkbox("Use depth for point brightness", &camera.useDepthOnPointBrightness);
        ImGui::Checkbox("Use shadow", &camera.useShadow);

        if (ImGui::CollapsingHeader("Orientation", ImGuiTreeNodeFlags_DefaultOpen)){
            ImGui::Checkbox("Show axes", &showAxes);
            ImGui::SliderFloat("Axis length", &axisLength, 0.1f, 100.0f, "%.1f", ImGuiSliderFlags_Logarithmic);
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
            if (ImGui::Button("Reset rotation")) modelRotation = glm::mat4(1.0f);
            ImGui::Separator();
        }

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
            ImGui::RadioButton("Cube shader", &c.shaderType, 3);
            ImGui::RadioButton("ColorGradient shader", &c.shaderType, 4);
            ImGui::RadioButton("debug shader", &c.shaderType, 5);
            ImGui::RadioButton("zMap Shader", &c.shaderType, 6);
            ImGui::RadioButton("Sphere shader", &c.shaderType, 7);
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
                    ImGui::TextDisabled("Cube edge length = PointSize in cm");
                    c.currentShader = &cubeShader;
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
                    // height (z) range of the cloud after applying the model rotation
                    float yMin = FLT_MAX, yMax = -FLT_MAX;
                    for (int k = 0; k < 8; k++){
                        glm::vec3 corner((k & 1) ? c.max.position.x : c.min.position.x,
                                         (k & 2) ? c.max.position.y : c.min.position.y,
                                         (k & 4) ? c.max.position.z : c.min.position.z);
                        float y = (modelMatrix * glm::vec4(corner, 1.0f)).z;
                        yMin = std::min(yMin, y);
                        yMax = std::max(yMax, y);
                    }
                    ImGui::SliderFloat("Start distance", &startDist, yMin, yMax);
                    ImGui::SliderFloat("End distance", &endDist, yMin, yMax);
                    
                    c.currentShader = &zmapShader;
                    break;
                }
                case 7:{
                    ImGui::TextDisabled("Sphere diameter = PointSize in cm");
                    c.currentShader = &sphereShader;
                    break;
                }
            }

            ImGui::NewLine();
            ImGui::PopID();
        }
            
        //ImGui::Text("%d Points", cloud.vertices.size());
        ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / io.Framerate, io.Framerate);
        replayButtonPressed = ImGui::Button("Replay");
        ImGui::SameLine();
        ImGui::Text("%d Positions", posPath.size());
        ImGui::SameLine();
        ImGui::Checkbox("with loop", &loop);
        ImGui::SameLine();
        ImGui::Checkbox("and record", &record);
        replayToEndButtonPressed = ImGui::Button("Replay from selection");
        ImGui::SameLine();
        replayNextButtonPressed = ImGui::Button("Replay one step");
        
        int i=0;
        static ImGuiTableFlags flags = ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg;
        if (ImGui::BeginTable("Waypoints", 8, flags)){
                        
            ImGui::TableSetupColumn("#");
            ImGui::TableSetupColumn("Pos x");
            ImGui::TableSetupColumn("Pos y");
            ImGui::TableSetupColumn("Pos z");
            ImGui::TableSetupColumn("Ori x");
            ImGui::TableSetupColumn("Ori y");
            ImGui::TableSetupColumn("Ori z");
            ImGui::TableHeadersRow();
            
            static int selection = false;
            for (glm::vec3 pos : posPath){
                ImGui::PushID(i);
                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0);
                selection = posPathSelection == i;
                if (ImGui::Selectable(std::to_string(i).c_str(), selection, 18)){
                    if (!ImGui::GetIO().KeyCtrl){
                        posPathSelection = i;
                        camera.Position = posPath[i];
                        camera.Orientation = oriPath[i];

                        //camera.Inputs(window);
                        camera.updateMatrix(45.0f, 0.1f, 500.0f);
                    }
                }

                ImGui::TableSetColumnIndex(1);
                ImGui::Text("%.3f", pos.x);
                ImGui::TableSetColumnIndex(2);
                ImGui::Text("%.3f", pos.y);
                ImGui::TableSetColumnIndex(3);
                ImGui::Text("%.3f", pos.z);
                ImGui::TableSetColumnIndex(4);
                ImGui::Text("%.3f", oriPath[i].x);
                ImGui::TableSetColumnIndex(5);
                ImGui::Text("%.3f", oriPath[i].y);
                ImGui::TableSetColumnIndex(6);
                ImGui::Text("%.3f", oriPath[i].z);
                ImGui::TableSetColumnIndex(7);
                if (ImGui::SmallButton("x")){wayPointToDelete = i;}
                ImGui::SameLine();
                if (ImGui::SmallButton("up")){wayPointUp = i;};
                ImGui::SameLine();
                if (ImGui::SmallButton("down")){wayPointDown = i;};
                
                i++;
                ImGui::PopID();
            }
            ImGui::EndTable();
            
        }
        
            //ImGui::PushID(i);
        ImGui::Text("%.1f", (float) wayPointToDelete);
        ImGui::Text("%.1f", (float) posPathSelection);

        ImGui::End();
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
                loadCloud(outPath);
                NFD_FreePathU8(outPath);
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
        // camera uses the window size for the aspect ratio and for centering the mouse
        {
            int win_w, win_h;
            glfwGetWindowSize(window, &win_w, &win_h);
            if (win_w > 0 && win_h > 0){
                camera.width = win_w;
                camera.height = win_h;
            }
        }
        //clear window
        glViewport(0, 0, display_w, display_h);
        glClearColor(clear_color.x * clear_color.w, clear_color.y * clear_color.w, clear_color.z * clear_color.w, clear_color.w);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        //set pointsize
        
        //camera rotation and movement update
        //process keypresses only if the curser isn't over the imgui menus
        if (!ImGui::GetIO().WantCaptureMouse){

        //Keyboard movement handling, independent of the frame rate
            float dt = ImGui::GetIO().DeltaTime;
            if (ImGui::IsKeyDown(ImGuiKey_W))
                camera.forward(dt);
            
            if (ImGui::IsKeyDown(ImGuiKey_A))
                camera.left(dt);

            if (ImGui::IsKeyDown(ImGuiKey_S))
                camera.backward(dt);

            if (ImGui::IsKeyDown(ImGuiKey_D))
                camera.right(dt);

            if (ImGui::IsKeyDown(ImGuiKey_R)){
                camera.up(dt);
            }
            if (ImGui::IsKeyDown(ImGuiKey_F)){
                camera.down(dt);
            }
            //Arrow keys: look around (alternative to the mouse), independent of the frame rate
            float keyRot = camera.keyRotSpeed * ImGui::GetIO().DeltaTime;
            if (ImGui::IsKeyDown(ImGuiKey_LeftArrow))
                camera.rotate(-keyRot, 0.0f);
            if (ImGui::IsKeyDown(ImGuiKey_RightArrow))
                camera.rotate(keyRot, 0.0f);
            if (ImGui::IsKeyDown(ImGuiKey_UpArrow))
                camera.rotate(0.0f, keyRot);
            if (ImGui::IsKeyDown(ImGuiKey_DownArrow))
                camera.rotate(0.0f, -keyRot);
            if (ImGui::IsKeyDown(ImGuiKey_C)){// && posPath.back() != camera.Position){
                if (posPath.size() == 0){
                    posPath.push_back(camera.Position);
                    oriPath.push_back(camera.Orientation);
                }
                else{
                    glm::vec3 lastPos = posPath.back();
                    glm::vec3 lastOri = oriPath.back();
                    if (lastPos.x != camera.Position.x ||
                        lastPos.y != camera.Position.y ||
                        lastPos.z != camera.Position.z ||
                        lastOri.x != camera.Orientation.x ||
                        lastOri.y != camera.Orientation.y ||
                        lastOri.z != camera.Orientation.z)
                        {
                        posPath.push_back(camera.Position);
                        oriPath.push_back(camera.Orientation);
                    }
                }
            }

            camera.Inputs(window);
            camera.updateMatrix(45.0f, 0.1f, 500.0f);
        }

        //delete waypoint to the "x" clicked row
        if (wayPointToDelete >= 0){
            posPath.erase(posPath.begin() + wayPointToDelete);
            oriPath.erase(oriPath.begin() + wayPointToDelete);
            //if the last waypoint was selected and deleted, select the new last
            if (wayPointToDelete <= posPathSelection){
                posPathSelection--;
            }
            //reset delete flag
            wayPointToDelete = -1;
        }

        //handle the waypoint up stuff. Up means lower index
        if (wayPointUp >= 0){
            if (wayPointUp >= 1){
                iter_swap(posPath.begin() + wayPointUp,
                          posPath.begin() + wayPointUp - 1);
                iter_swap(oriPath.begin() + wayPointUp,
                          oriPath.begin() + wayPointUp - 1);
            }
            //reset the flag
            wayPointUp = -1;
        }

        //handle waypoint down stuff. Down means higher index
        if (wayPointDown >= 0){
            if (wayPointDown < posPath.size()-1){
                iter_swap(posPath.begin() + wayPointDown,
                posPath.begin() + wayPointDown + 1);
                iter_swap(oriPath.begin() + wayPointDown,
                oriPath.begin() + wayPointDown + 1);
            }
            //reset the flag
            wayPointDown = -1;
        }


        int replayStart = 0;
        int replayEnd = posPath.size() - 1;
        
        if (replayToEndButtonPressed || replayNextButtonPressed){
            // no (valid) selection: start at the first waypoint
            replayStart = (posPathSelection >= 0 && posPathSelection <= replayEnd) ? posPathSelection : 0;
        }
        if (replayNextButtonPressed && replayStart < replayEnd){
            replayEnd = replayStart + 1;
        }

        if ((replayButtonPressed || replayToEndButtonPressed || replayNextButtonPressed) && posPath.size()>1){
            //state handling
            replayButtonPressed = false;
            currentReplayPosition = 0;
            
            //path interpolation
            

            //init and malloc variables
            int codec = cv::VideoWriter::fourcc('M', 'J', 'P', 'G');
            double fps = 25.0;
            time_t timestamp;
            char output[50];
            struct tm* datetime;
            time(&timestamp);
            datetime = localtime(&timestamp);
            strftime(output, 50, "%Y_%m_%d_%H_%M_%S", datetime);
            std::string buffer(output);
            std::string filename = "../../records/record_" + buffer + ".avi";
            // record with the current framebuffer size (the window may have been resized)
            recordSize = cv::Size(display_w, display_h);
            if (record){
                std::error_code ec;
                std::filesystem::create_directories("../../records", ec);
                //try to open the videowriter; on failure replay without recording
                writer.open(filename, codec, fps, recordSize, true);
                if (!writer.isOpened()) {
                    std::cerr << "Could not open the output video file for write: " << filename << std::endl;
                    record = false;
                }
                else
                    std::cout << "Recording to " << filename << std::endl;
            }
            
            // Interpolate between waypoints: at most framePathDist meters and frameAngleDeg degrees per frame
            const float framePathDist = 0.1f;
            const float frameAngleDeg = 2.0f;
            posPathInterpolated.clear();
            oriPathInterpolated.clear();
            auto appendSegment = [&](const glm::vec3& p0, const glm::vec3& o0, const glm::vec3& p1, const glm::vec3& o1){
                glm::vec3 n0 = glm::normalize(o0);
                glm::vec3 n1 = glm::normalize(o1);
                float angleDeg = glm::degrees(std::acos(glm::clamp(glm::dot(n0, n1), -1.0f, 1.0f)));
                int steps = std::max({1,
                                      (int)std::ceil(glm::length(p1 - p0) / framePathDist),
                                      (int)std::ceil(angleDeg / frameAngleDeg)});
                for (int ip = 0; ip < steps; ip++){
                    float t = (float)ip / steps;
                    posPathInterpolated.push_back(glm::mix(p0, p1, t));
                    glm::vec3 o = glm::mix(n0, n1, t);
                    // opposite directions would give a zero vector halfway: keep the previous one
                    oriPathInterpolated.push_back(glm::length(o) > 1e-4f ? glm::normalize(o) : n0);
                }
            };
            for (int i=replayStart; i<replayEnd; i++)
                appendSegment(posPath[i], oriPath[i], posPath[i+1], oriPath[i+1]);
            if (loop)
                appendSegment(posPath.back(), oriPath.back(), posPath.front(), oriPath.front());
            else{
                // also show the last waypoint itself
                posPathInterpolated.push_back(posPath[replayEnd]);
                oriPathInterpolated.push_back(glm::normalize(oriPath[replayEnd]));
            }

        }
        else{
            replayButtonPressed = false;
        }

        if (currentReplayPosition != -1){
            //camera.Position = posPath[currentReplayPosition];
            camera.Position = posPathInterpolated[currentReplayPosition];
            camera.Orientation = oriPathInterpolated[currentReplayPosition];
            camera.Inputs(window);
            camera.updateMatrix(45.0f, 0.1f, 500.0f);
            if (currentReplayPosition == posPathInterpolated.size()-1){
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

        // Pivot for the model rotation: center of the bounding box of all loaded clouds
        if (!clouds.empty()){
            glm::vec3 lo = clouds[0].min.position;
            glm::vec3 hi = clouds[0].max.position;
            for (auto & c : clouds){
                lo = glm::min(lo, c.min.position);
                hi = glm::max(hi, c.max.position);
            }
            modelCenter = (lo + hi) * 0.5f;
        }
        modelMatrix = glm::translate(glm::mat4(1.0f), modelCenter) * modelRotation * glm::translate(glm::mat4(1.0f), -modelCenter);

        for (auto & c : clouds){
            glPointSize(c.pointssize);

            c.currentShader->Activate();

            camera.Matrix(*c.currentShader, "camMatrix");
            glUniformMatrix4fv(glGetUniformLocation(c.currentShader->ID, "model"), 1, GL_FALSE, glm::value_ptr(modelMatrix));

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
                case 3:
                case 7:{
                    // Ray-cast cube/sphere: sprite size is computed per point in the vertex shader
                    glEnable(GL_PROGRAM_POINT_SIZE);
                    GLuint id = c.currentShader->ID;
                    glm::mat4 invCam = glm::inverse(camera.cameraMatrix);
                    float projScale = (float)display_h / std::tan(glm::radians(45.0f) / 2.0f);
                    // world size follows the PointSize slider: 1 = 1 cm
                    float worldSize = c.pointssize * 0.01f;
                    glUniform1f(glGetUniformLocation(id, c.shaderType == 3 ? "cubeSize" : "sphereSize"), worldSize);
                    glUniform1f(glGetUniformLocation(id, "projScale"), projScale);
                    glUniform2f(glGetUniformLocation(id, "viewportSize"), (float)display_w, (float)display_h);
                    glUniformMatrix4fv(glGetUniformLocation(id, "invCamMatrix"), 1, GL_FALSE, glm::value_ptr(invCam));
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
            glDisable(GL_PROGRAM_POINT_SIZE);
        }

        // Coordinate axes with labels (hidden while a tracking shot is replayed/recorded)
        if (showAxes && currentReplayPosition < 0){
            glm::mat4 axisModel = glm::translate(glm::mat4(1.0f), modelCenter) * glm::scale(glm::mat4(1.0f), glm::vec3(axisLength));
            axisShader.Activate();
            glUniformMatrix4fv(glGetUniformLocation(axisShader.ID, "camMatrix"), 1, GL_FALSE, glm::value_ptr(camera.cameraMatrix));
            glUniformMatrix4fv(glGetUniformLocation(axisShader.ID, "model"), 1, GL_FALSE, glm::value_ptr(axisModel));
            glDisable(GL_DEPTH_TEST);   // always on top of the points
            axisVAO.Bind();
            glLineWidth(3.0f);
            glDrawArrays(GL_LINES, 0, 6);
            glLineWidth(1.0f);
            axisVAO.Unbind();
            glEnable(GL_DEPTH_TEST);

            ImDrawList* drawList = ImGui::GetForegroundDrawList();
            ImVec2 displaySize = ImGui::GetIO().DisplaySize;
            const char* labels[3] = {"X", "Y", "Z"};
            const ImU32 labelColors[3] = {IM_COL32(255, 60, 60, 255), IM_COL32(60, 255, 60, 255), IM_COL32(90, 140, 255, 255)};
            for (int a = 0; a < 3; a++){
                glm::vec3 tip = modelCenter;
                tip[a] += axisLength * 1.1f;
                glm::vec4 clip = camera.cameraMatrix * glm::vec4(tip, 1.0f);
                if (clip.w <= 0.0f) continue;   // behind the camera
                glm::vec3 ndc = glm::vec3(clip) / clip.w;
                ImVec2 p((ndc.x * 0.5f + 0.5f) * displaySize.x, (0.5f - ndc.y * 0.5f) * displaySize.y);
                float fontSize = ImGui::GetFontSize() * 1.5f;
                drawList->AddText(ImGui::GetFont(), fontSize, ImVec2(p.x - fontSize * 0.25f, p.y - fontSize * 0.5f), labelColors[a], labels[a]);
            }
        }

        ImGui::Render();

        // Don't draw the imGui stuff when the the tracking shot is beeing taken
        if (currentReplayPosition < 0){
            ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        }
        
        glfwSwapBuffers(window);

        if (currentReplayPosition != -1 && record){
            //glReadPixels(0, 0, width, height, FORMAT, GL_UNSIGNED_BYTE, pixels);
            frame.create(recordSize.height, recordSize.width, CV_8UC3);

            glPixelStorei(GL_PACK_ALIGNMENT, (frame.step & 3) ? 1 : 4);
            glPixelStorei(GL_PACK_ROW_LENGTH, frame.step/frame.elemSize());
            glReadPixels(0, 0, frame.cols, frame.rows, GL_BGR, GL_UNSIGNED_BYTE, frame.data);

            cv::Mat flipped;
            cv::flip(frame, flipped, 0);
            writer.write(flipped);
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
