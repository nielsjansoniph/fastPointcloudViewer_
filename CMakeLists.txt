cmake_minimum_required(VERSION 3.14)
project(fpcv VERSION 0.1.0 LANGUAGES C CXX)
set(CMAKE_CXX_STANDARD 17)

find_package(PCL REQUIRED COMPONENTS common io)
find_package(OpenCV REQUIRED COMPONENTS core videoio)
find_package(assimp CONFIG REQUIRED)

if(WIN32)
    # Windows: dependencies via vcpkg
    find_package(octomap REQUIRED)
    find_package(glad REQUIRED)
    find_package(glfw3 CONFIG REQUIRED)
    find_package(imgui CONFIG REQUIRED)
    find_package(nfd CONFIG REQUIRED)
    set(FPCV_GUI_LIBS
        octomap octomath octomap-static octomath-static
        glad::glad glfw imgui::imgui nfd::nfd)
else()
    # Linux (e.g. Ubuntu 22.04): PCL, OpenCV, assimp, glm from apt;
    # GLFW, glad, Dear ImGui and nativefiledialog-extended are fetched here.
    include(FetchContent)

    set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
    set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
    set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
    set(GLAD_PROFILE "core" CACHE STRING "" FORCE)
    set(GLAD_API "gl=3.3" CACHE STRING "" FORCE)
    set(NFD_PORTAL ON CACHE BOOL "Use xdg-desktop-portal instead of GTK" FORCE)

    FetchContent_Declare(glfw  GIT_REPOSITORY https://github.com/glfw/glfw.git GIT_TAG 3.4)
    FetchContent_Declare(glad  GIT_REPOSITORY https://github.com/Dav1dde/glad.git GIT_TAG v0.1.36)
    FetchContent_Declare(nfd   GIT_REPOSITORY https://github.com/btzy/nativefiledialog-extended.git GIT_TAG v1.2.1)
    FetchContent_Declare(imgui GIT_REPOSITORY https://github.com/ocornut/imgui.git GIT_TAG v1.91.5)
    FetchContent_MakeAvailable(glfw glad nfd)

    FetchContent_GetProperties(imgui)
    if(NOT imgui_POPULATED)
        FetchContent_Populate(imgui)
    endif()
    add_library(imgui STATIC
        ${imgui_SOURCE_DIR}/imgui.cpp
        ${imgui_SOURCE_DIR}/imgui_draw.cpp
        ${imgui_SOURCE_DIR}/imgui_tables.cpp
        ${imgui_SOURCE_DIR}/imgui_widgets.cpp
        ${imgui_SOURCE_DIR}/backends/imgui_impl_glfw.cpp
        ${imgui_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp)
    target_include_directories(imgui PUBLIC ${imgui_SOURCE_DIR} ${imgui_SOURCE_DIR}/backends)
    target_link_libraries(imgui PUBLIC glfw)

    find_package(OpenGL REQUIRED)
    set(FPCV_GUI_LIBS glad glfw imgui nfd OpenGL::GL ${CMAKE_DL_LIBS})
endif()

add_executable(fpcv main.cpp stb.cpp Texture.cpp shaderClass.cpp Camera.cpp Cloud.cpp VBO.cpp VAO.cpp EBO.cpp Mesh.cpp)

target_include_directories(fpcv PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} ${PCL_INCLUDE_DIRS})
target_compile_definitions(fpcv PRIVATE ${PCL_DEFINITIONS})

target_link_libraries(fpcv
    ${FPCV_GUI_LIBS}
    ${PCL_LIBRARIES}
    assimp::assimp
    ${OpenCV_LIBS}
)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
