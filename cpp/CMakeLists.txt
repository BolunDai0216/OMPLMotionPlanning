cmake_minimum_required(VERSION 3.10)
project(interview)

# Specify the C++ standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Find required packages
find_package(ompl REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)
find_package(hpp-fcl REQUIRED)
find_package(yaml-cpp REQUIRED)

# Include directories
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${PROJECT_SOURCE_DIR}/include)

# Add executable target with source files
add_executable(interview src/main.cpp src/state_checker.cpp src/goal_checker.cpp)
add_executable(preplan src/preplan.cpp src/state_checker.cpp src/goal_checker.cpp)
add_executable(plan src/plan.cpp src/state_checker.cpp src/goal_checker.cpp)

# Link libraries
target_link_libraries(interview ${OMPL_LIBRARIES} pinocchio::pinocchio hpp-fcl::hpp-fcl yaml-cpp)
target_link_libraries(preplan ${OMPL_LIBRARIES} pinocchio::pinocchio hpp-fcl::hpp-fcl yaml-cpp)
target_link_libraries(plan ${OMPL_LIBRARIES} pinocchio::pinocchio hpp-fcl::hpp-fcl yaml-cpp)
