cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(plane_segmentation)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(PCL 1.2 REQUIRED)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


add_executable (plane_segmentation main.cpp Model.cpp Plane.cpp Point.cpp kdtree.cpp Matrix.cpp PointIO.cpp 3dKDtree.cpp PlaneFitting.cpp Segmentation.cpp)
target_link_libraries (plane_segmentation ${PCL_LIBRARIES})


