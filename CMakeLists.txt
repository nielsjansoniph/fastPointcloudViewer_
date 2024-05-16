cmake_minimum_required(VERSION 3.0.0)
project(fpcv VERSION 0.1.0 LANGUAGES C CXX)

find_package(octomap REQUIRED)
find_package(pcl REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(glad REQUIRED)
find_package(glfw3 CONFIG REQUIRED)
find_package(imgui CONFIG REQUIRED)

add_executable(fpcv main.cpp shaderClass.cpp Camera.cpp Cloud.cpp VBO.cpp VAO.cpp)


target_link_libraries(fpcv
    octomap octomath octomap-static octomath-static
    ${PCL_LIBRARIES}
    glad::glad
    glfw
    imgui::imgui
)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)

