cmake_minimum_required(VERSION 3.1.0)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/../cmake)
include(DefaultBuildType)

# ==== Project Name ====
project(ouster_ros)

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
find_package(Eigen3 REQUIRED)
find_package(GLEW REQUIRED)
find_package(glfw3 REQUIRED)

find_package(
  catkin REQUIRED
  COMPONENTS message_generation
  std_msgs
  mrs_msgs
  sensor_msgs
  geometry_msgs
  pcl_ros
  pcl_conversions
  roscpp
  tf2
  tf2_ros
  tf2_geometry_msgs
  nodelet)

# ==== Options ====

# Point Cloud Library: Override CXX flags inherited from catkin workspace, if precompiled PCL binaries from Debian repositories are used
if (DEFINED ENV{PCL_CROSS_COMPILATION})
  set(PCL_CROSS_COMPILATION $ENV{PCL_CROSS_COMPILATION})
else()
  set(PCL_CROSS_COMPILATION "false")
endif()
if(${PCL_CROSS_COMPILATION} STREQUAL "false")
  message("Using precompiled PCL binaries from Debian repositories. Disabling flag -march=native by overriding catkin workspace CMAKE_CXX_FLAGS.")
  set(CMAKE_CXX_FLAGS "-std=c++17")
else()
  message("Using manually compiled PCL binaries. Inheriting all CMAKE_CXX_FLAGS from catkin workspace.")
endif()

# add_compile_options(-Wall -Wextra)
set(CMAKE_CXX_STANDARD 17)
option(CMAKE_POSITION_INDEPENDENT_CODE "Build position independent code." ON)

# ==== Catkin ====
add_message_files(FILES PacketMsg.msg)
add_service_files(FILES OSConfigSrv.srv)
generate_messages(DEPENDENCIES std_msgs sensor_msgs geometry_msgs)

set(_ouster_ros_INCLUDE_DIRS "include;../ouster_client/include;../ouster_viz/include")

catkin_package(
  INCLUDE_DIRS
  ${_ouster_ros_INCLUDE_DIRS}

  LIBRARIES
  ouster_ros
  OusterNodelet
  OusterCloudNodelet
  OusterImgNodelet

  CATKIN_DEPENDS
  roscpp
  message_runtime
  pcl_ros
  std_msgs
  mrs_msgs
  sensor_msgs
  geometry_msgs
  DEPENDS
  EIGEN3 GLFW3 GLEW)

# ==== Libraries ====
# Build static libraries and bundle them into ouster_ros using the `--whole-archive` flag. This is
# necessary because catkin doesn't interoperate easily with target-based cmake builds. Object
# libraries are the recommended way to do this, but require >=3.13 to propagate usage requirements.
set(_SAVE_BUILD_SHARED_LIBS ${BUILD_SHARED_LIBS})
set(BUILD_SHARED_LIBS OFF)
add_subdirectory(../ouster_client ouster_client EXCLUDE_FROM_ALL)
add_subdirectory(../ouster_viz ouster_viz EXCLUDE_FROM_ALL)
set(BUILD_SHARED_LIBS ${_SAVE_BUILD_SHARED_LIBS})

# catkin adds all include dirs to a single variable, don't try to use targets
include_directories(${_ouster_ros_INCLUDE_DIRS} SYSTEM ${catkin_INCLUDE_DIRS})

add_library(ouster_ros src/ros.cpp)
target_link_libraries(ouster_ros PUBLIC ${catkin_LIBRARIES} ouster_build PRIVATE
  -Wl,--whole-archive ouster_client ouster_viz -Wl,--no-whole-archive)
add_dependencies(ouster_ros ${PROJECT_NAME}_gencpp)

# ==== Executables ====
add_library(OusterCloudNodelet src/os_cloud_nodelet.cpp)
target_link_libraries(OusterCloudNodelet ouster_ros ${catkin_LIBRARIES})
add_dependencies(OusterCloudNodelet ${PROJECT_NAME}_gencpp)

add_library(OusterNodelet src/os_nodelet.cpp)
target_link_libraries(OusterNodelet ouster_ros ${catkin_LIBRARIES})
add_dependencies(OusterNodelet ${PROJECT_NAME}_gencpp)

add_library(OusterImgNodelet src/img_nodelet.cpp)
target_link_libraries(OusterImgNodelet ouster_ros ${catkin_LIBRARIES})
add_dependencies(OusterImgNodelet ${PROJECT_NAME}_gencpp)

# # ==== Install ====
# install(
#   TARGETS ouster_ros OusterNodelet OusterCloudNodelet viz_node img_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
# install(
#   DIRECTORY ${_ouster_ros_INCLUDE_DIRS}
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
# install(FILES ouster.launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
