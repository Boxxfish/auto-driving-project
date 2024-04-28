cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(icp)

find_package(PCL 1.5 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(SOURCES visualizer.cpp ground_registration.cpp dataset.cpp visualizer.cpp frame.cpp)
add_executable (icp icp.cpp)
add_executable (compute_baselines compute_baselines.cpp)

target_sources(icp PUBLIC ${SOURCES})
target_link_libraries (icp ${PCL_LIBRARIES})
target_link_libraries (compute_baselines ${PCL_LIBRARIES})