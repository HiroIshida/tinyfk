cmake_minimum_required( VERSION 2.8 FATAL_ERROR )
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_CXX_STANDARD 14)
set (URDF_MAJOR_VERSION 1)
set (URDF_MINOR_VERSION 0)
set (URDF_PATCH_VERSION 4)

set (URDF_VERSION ${URDF_MAJOR_VERSION}.${URDF_MINOR_VERSION}.${URDF_PATCH_VERSION})
set (URDF_MAJOR_MINOR_VERSION ${URDF_MAJOR_VERSION}.${URDF_MINOR_VERSION})

add_subdirectory("external/tinyxml")

find_package(console_bridge REQUIRED)
include_directories(SYSTEM ${console_bridge_INCLUDE_DIRS})
link_directories(${console_bridge_LIBRARY_DIRS})

include_directories("external/tinyxml")
include_directories("include")
link_directories("external/tinyxml")

add_library(urdfdom_model STATIC src/pose.cpp src/model.cpp src/link.cpp src/joint.cpp)
target_link_libraries(urdfdom_model tinyxml ${console_bridge_LIBRARIES})
set_target_properties(urdfdom_model PROPERTIES SOVERSION ${URDF_MAJOR_MINOR_VERSION})
