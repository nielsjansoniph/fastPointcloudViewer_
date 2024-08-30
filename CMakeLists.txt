cmake_minimum_required(VERSION 3.5.0)
project(fpcv VERSION 0.1.0 LANGUAGES C CXX)
set(CMAKE_CXX_STANDARD 17)

find_package(octomap REQUIRED)
find_package(pcl REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(glad REQUIRED)
find_package(glfw3 CONFIG REQUIRED)
find_package(imgui CONFIG REQUIRED)
find_package(assimp CONFIG REQUIRED)
#message("${PCL_LIBRARIES}=" ${PCL_LIBRARIES})

add_executable(fpcv main.cpp stb.cpp Texture.cpp shaderClass.cpp Camera.cpp Cloud.cpp VBO.cpp VAO.cpp EBO.cpp Mesh.cpp)


target_link_libraries(fpcv
    octomap octomath octomap-static octomath-static
    ${PCL_LIBRARIES}
    glad::glad
    glfw
    imgui::imgui
    assimp::assimp
)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)

