cmake_minimum_required(VERSION 3.1.0)

# ==== Project Name ====
project(ouster_viz)

# Override CXX flags inherited from workspace, if precompiled PCL binaries from debian repos are used
if (DEFINED ENV{PCL_CROSS_COMPILATION})
  set(PCL_CROSS_COMPILATION $ENV{PCL_CROSS_COMPILATION})
else()
  set(PCL_CROSS_COMPILATION "false")
endif()
if(${PCL_CROSS_COMPILATION} STREQUAL "false")
  message("Using precompiled PCL binaries from debian repos. Overriding catkin workspace CMAKE_CXX_FLAGS.")
  set(CMAKE_CXX_FLAGS "-std=c++17")
else()
  message("Using custom-built PCL binaries. Inheriting all CMAKE_CXX_FLAGS from catkin workspace.")
endif()

# ==== Requirements ====
set(OpenGL_GL_PREFERENCE GLVND)
find_package(OpenGL REQUIRED)
find_package(Threads)
find_package(GLEW REQUIRED)
find_package(glfw3 CONFIG REQUIRED)

# ==== Libraries ====
add_library(ouster_viz src/point_viz.cpp src/cloud.cpp src/camera.cpp src/image.cpp)
target_link_libraries(ouster_viz
  PUBLIC Threads::Threads glfw GLEW::GLEW OpenGL::GL
  PRIVATE ouster_client)
target_include_directories(ouster_viz PUBLIC include)

# ==== Executables ====
add_executable(simple_viz src/main.cpp)
target_link_libraries(simple_viz ouster_client ouster_viz)
