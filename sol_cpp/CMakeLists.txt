cmake_minimum_required(VERSION 3.10)
project(ompl_hello_world)

# Specify the C++ standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_INSTALL_PREFIX $ENV{CONDA_PREFIX})

# Include directories for Eigen
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${PROJECT_SOURCE_DIR}/include)

# Find the OMPL package
find_package(ompl REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)
find_package(hpp-fcl REQUIRED)

# Add executable target with source files
add_executable(ompl_hello_world src/main.cpp)

# Link OMPL libraries
target_link_libraries(ompl_hello_world ${OMPL_LIBRARIES} pinocchio::pinocchio hpp-fcl::hpp-fcl)
