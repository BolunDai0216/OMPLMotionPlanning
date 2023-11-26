cmake_minimum_required(VERSION 3.5)
project(ompl_hello_world)

set(CMAKE_INSTALL_PREFIX $ENV{CONDA_PREFIX})

# Find the OMPL package
find_package(ompl REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)
# find_package(hpp-fcl REQUIRED)

# Add executable target with source files
add_executable(ompl_hello_world test.cpp)

# Include directories for Eigen
include_directories(${EIGEN3_INCLUDE_DIR})

# Link OMPL libraries
target_link_libraries(ompl_hello_world ${OMPL_LIBRARIES})
