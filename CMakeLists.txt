cmake_minimum_required(VERSION 3.10)
project(g1_state_estimator)

set(CMAKE_CXX_STANDARD 17)

# Dependencies
# find_package(Eigen3 REQUIRED) # System Eigen
set(EIGEN3_INCLUDE_DIR "${CMAKE_SOURCE_DIR}/thirdparty/eigen")
include_directories(${EIGEN3_INCLUDE_DIR})

find_package(realsense2 QUIET) # Optional on Windows build if not installed
# find_package(livox_sdk2 REQUIRED) 

# Unitree SDK2 Integration
set(UNITREE_SDK2_PATH "${CMAKE_SOURCE_DIR}/unitree_sdk2")
include_directories(${UNITREE_SDK2_PATH}/include)
link_directories(${UNITREE_SDK2_PATH}/lib)

# Source Files
file(GLOB SOURCES "src/*.cpp")

# Executable
add_executable(g1_state_estimator ${SOURCES})

# Linking
target_link_libraries(g1_state_estimator 
    unitree_sdk2 
    # ${realsense2_LIBRARY}
    # livox_sdk2_static
    # Eigen3::Eigen
    pthread
)

# Install Instructions (optional)
# install(TARGETS g1_state_estimator DESTINATION bin)
