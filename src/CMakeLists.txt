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

# Unitree SDK2 (adjust path as needed)
set(UNITREE_SDK_PATH "/opt/unitree_sdk2" CACHE PATH "Path to Unitree SDK2")
include_directories(${UNITREE_SDK_PATH}/include)
link_directories(${UNITREE_SDK_PATH}/lib)

# Livox SDK (adjust path as needed)
set(LIVOX_SDK_PATH "/opt/livox_sdk" CACHE PATH "Path to Livox SDK")
include_directories(${LIVOX_SDK_PATH}/include)
link_directories(${LIVOX_SDK_PATH}/lib)

# Include directories
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${EIGEN3_INCLUDE_DIR}
)

# Source files
set(SOURCES
    ICP2D.cpp
    ScanMatcher.cpp
    PoseGraph.cpp
    LivoxInterface.cpp
    UDPPublisher.cpp
    G1Localizer.cpp
)

# Library
add_library(g1_localization_lib SHARED ${SOURCES})
target_link_libraries(g1_localization_lib
    unitree_sdk2
    livox_sdk_static
    pthread
)

# Executable
add_executable(g1_localization_node g1_localization_node.cpp)
target_link_libraries(g1_localization_node
    g1_localization_lib
)

# Install
install(TARGETS g1_localization_node g1_localization_lib
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
)

install(FILES
    ICP2D.hpp
    ScanMatcher.hpp
    PoseGraph.hpp
    G1Localizer.hpp
    LivoxInterface.hpp
    UDPPublisher.hpp
    DESTINATION include/g1_localization
)
