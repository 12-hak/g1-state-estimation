cmake_minimum_required(VERSION 3.10)
project(g1_localization)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)

# Optimization flags
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -march=native -DNDEBUG")

# Find Eigen3 (try multiple methods)
find_package(Eigen3 QUIET)
if(NOT Eigen3_FOUND)
    # Try pkg-config
    find_package(PkgConfig QUIET)
    if(PKG_CONFIG_FOUND)
        pkg_check_modules(EIGEN3 QUIET eigen3)
    endif()
    
    # Fallback to common install locations
    if(NOT EIGEN3_FOUND)
        set(EIGEN3_INCLUDE_DIR "/usr/include/eigen3" "/usr/local/include/eigen3")
        message(STATUS "Using fallback Eigen3 path: ${EIGEN3_INCLUDE_DIR}")
    endif()
else()
    set(EIGEN3_INCLUDE_DIR ${EIGEN3_INCLUDE_DIR})
endif()

# Unitree SDK2
set(UNITREE_SDK_PATH "/home/unitree/development/unitree_sdk2")
message(STATUS "Using Unitree SDK2 path: ${UNITREE_SDK_PATH}")
include_directories(${UNITREE_SDK_PATH}/include)
link_directories(${UNITREE_SDK_PATH}/lib)

# CycloneDDS (from Unitree Robotics)
set(CYCLONEDDS_PATH "/opt/unitree_robotics")
message(STATUS "Using CycloneDDS path: ${CYCLONEDDS_PATH}")
include_directories(${CYCLONEDDS_PATH}/include)
include_directories(${CYCLONEDDS_PATH}/include/ddscxx)
# Set RPATH to include SDK and CycloneDDS libraries for runtime loading
# Use \$ORIGIN (escaped) so CMake doesn't try to expand it as a variable
set(CMAKE_INSTALL_RPATH "${CYCLONEDDS_PATH}/lib;${UNITREE_SDK_PATH}/lib;\$ORIGIN")
set(CMAKE_BUILD_RPATH "${CYCLONEDDS_PATH}/lib;${UNITREE_SDK_PATH}/lib;\$ORIGIN")
set(CMAKE_SKIP_BUILD_RPATH FALSE)
set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

# Include directories
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${EIGEN3_INCLUDE_DIR}
)

# Source files
set(SOURCES
    src/ICP2D.cpp
    src/ScanMatcher.cpp
    src/LivoxInterface.cpp
    src/UDPPublisher.cpp
    src/G1Localizer.cpp
)

# Library
add_library(g1_localization_lib SHARED ${SOURCES})
target_link_libraries(g1_localization_lib
    unitree_sdk2
    ${CYCLONEDDS_PATH}/lib/libddsc.so
    ${CYCLONEDDS_PATH}/lib/libddscxx.so
    pthread
    livox_lidar_sdk_shared
)

# Executable
add_executable(g1_localization_node src/g1_localization_node.cpp)
target_link_libraries(g1_localization_node
    g1_localization_lib
)

# New Visualizer Tool
add_executable(g1_localizer_viz src/g1_localizer_viz.cpp)
target_link_libraries(g1_localizer_viz
    g1_localization_lib
)

# LiDAR Recorder (standalone)
add_executable(g1_lidar_recorder 
    src/g1_lidar_recorder.cpp
    src/LiDARRecorder.cpp
    src/LivoxInterface.cpp
)
target_link_libraries(g1_lidar_recorder
    livox_lidar_sdk_shared
    pthread
)

# Install
install(TARGETS g1_localization_node g1_localization_lib
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
)

install(FILES
    src/ICP2D.hpp
    src/ScanMatcher.hpp
    src/G1Localizer.hpp
    src/LivoxInterface.hpp
    src/UDPPublisher.hpp
    DESTINATION include/g1_localization
)
