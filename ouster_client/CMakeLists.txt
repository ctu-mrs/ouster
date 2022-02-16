cmake_minimum_required(VERSION 3.1.0)

# ==== Project Name ====
project(ouster_client)
set(ouster_client_VERSION_SUFFIX "")

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
find_package(jsoncpp REQUIRED)

find_package(
  catkin REQUIRED COMPONENTS
  roscpp
  )

set(LIBRARIES
  ouster_client
  )

set(EXECUTABLES
  ouster_client_example
  )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${LIBRARIES}
  CATKIN_DEPENDS roscpp
  )

# ==== Build Metadata Target ====
add_custom_target(generate_build_header)
add_custom_command(TARGET generate_build_header PRE_BUILD
  COMMAND ${CMAKE_COMMAND} -DVERSION_GEN_OUT_DIR="${CMAKE_CURRENT_BINARY_DIR}/include/ouster"
  -DVERSION_GEN_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/.."
  -DBUILD_TYPE="${CMAKE_BUILD_TYPE}"
  -DBUILD_SYSTEM="${CMAKE_SYSTEM_NAME}"
  -Douster_client_VERSION="${ouster_client_VERSION}"
  -Douster_client_VERSION_SUFFIX="${ouster_client_VERSION_SUFFIX}"
  -P ${CMAKE_CURRENT_SOURCE_DIR}/../cmake/VersionGen.cmake)

add_library(ouster_build INTERFACE)
target_include_directories(ouster_build INTERFACE ${CMAKE_CURRENT_BINARY_DIR}/include ${catkin_INCLUDE_DIRS})
add_dependencies(ouster_build generate_build_header)

# ==== Libraries ====
add_library(ouster_client src/client.cpp src/types.cpp src/netcompat.cpp src/lidar_scan.cpp)
target_link_libraries(ouster_client PUBLIC jsoncpp_lib Eigen3::Eigen ouster_build)
if(WIN32)
  target_link_libraries(ouster_client PUBLIC ws2_32)
endif()
target_include_directories(ouster_client PUBLIC include ${catkin_INCLUDE_DIRS})

# ==== Executables ====
add_executable(ouster_client_example src/example.cpp)
target_link_libraries(ouster_client_example ouster_client ouster_build)

## --------------------------------------------------------------
## |                           Install                          |
## --------------------------------------------------------------

install(TARGETS ${LIBRARIES}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
  )

install(TARGETS ${EXECUTABLES}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(DIRECTORY include/ ${CMAKE_CURRENT_BINARY_DIR}/include/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
  )

install(DIRECTORY ./
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING PATTERN "*.xml"
  )
