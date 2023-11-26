cmake_minimum_required(VERSION 3.5)
project(ompl_hello_world)

# Find the OMPL package
find_package(ompl REQUIRED)
find_package(Eigen3 REQUIRED)

# Add executable target with source files
add_executable(ompl_hello_world test.cpp)

# Include directories for Eigen
include_directories(${EIGEN3_INCLUDE_DIR})

# Link OMPL libraries
target_link_libraries(ompl_hello_world ${OMPL_LIBRARIES})
