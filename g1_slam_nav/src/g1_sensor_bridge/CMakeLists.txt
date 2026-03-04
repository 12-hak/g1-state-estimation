cmake_minimum_required(VERSION 3.8)
project(g1_sensor_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(g1_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# Unitree SDK2 paths (configurable via environment or cmake args)
set(UNITREE_SDK_PATH "$ENV{UNITREE_SDK_PATH}" CACHE PATH "Path to unitree_sdk2")
if(NOT UNITREE_SDK_PATH)
  set(UNITREE_SDK_PATH "/home/unitree/development/unitree_sdk2")
endif()

set(CYCLONEDDS_PATH "$ENV{CYCLONEDDS_PATH}" CACHE PATH "Path to CycloneDDS")
if(NOT CYCLONEDDS_PATH)
  set(CYCLONEDDS_PATH "/opt/unitree_robotics")
endif()

# Livox SDK path (optional: only build livox_bridge_node when found)
set(LIVOX_SDK_PATH "$ENV{LIVOX_SDK_PATH}" CACHE PATH "Path to Livox SDK2")
if(LIVOX_SDK_PATH)
  find_path(LIVOX_INCLUDE_DIR NAMES livox_lidar_def.h PATHS "${LIVOX_SDK_PATH}/include" NO_DEFAULT_PATH)
  find_library(LIVOX_LIB NAMES livox_lidar_sdk_shared PATHS "${LIVOX_SDK_PATH}/lib" NO_DEFAULT_PATH)
endif()
set(HAVE_LIVOX_SDK ${LIVOX_INCLUDE_DIR})

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${UNITREE_SDK_PATH}/include
  ${CYCLONEDDS_PATH}/include
  ${CYCLONEDDS_PATH}/include/ddscxx
)
if(HAVE_LIVOX_SDK)
  include_directories(${LIVOX_SDK_PATH}/include)
endif()

link_directories(
  ${UNITREE_SDK_PATH}/lib
  ${CYCLONEDDS_PATH}/lib
)
if(HAVE_LIVOX_SDK)
  link_directories(${LIVOX_SDK_PATH}/lib)
endif()

# --- Livox Bridge Node (only when Livox SDK is available) ---
if(HAVE_LIVOX_SDK)
  add_executable(livox_bridge_node
    src/livox_bridge_node.cpp
  )
  ament_target_dependencies(livox_bridge_node
    rclcpp sensor_msgs geometry_msgs tf2 tf2_ros
  )
  target_link_libraries(livox_bridge_node
    livox_lidar_sdk_shared
    Eigen3::Eigen
  )
  list(APPEND G1_SENSOR_BRIDGE_TARGETS livox_bridge_node)
else()
  message(STATUS "Livox SDK not found: livox_bridge_node will not be built")
endif()

# --- Unitree Bridge Node ---
add_executable(unitree_bridge_node
  src/unitree_bridge_node.cpp
)
ament_target_dependencies(unitree_bridge_node
  rclcpp sensor_msgs geometry_msgs g1_msgs tf2 tf2_ros
)
target_link_libraries(unitree_bridge_node
  unitree_sdk2
  ${CYCLONEDDS_PATH}/lib/libddsc.so
  ${CYCLONEDDS_PATH}/lib/libddscxx.so
  Eigen3::Eigen
)

# --- Leg Odometry Node ---
add_executable(leg_odometry_node
  src/leg_odometry_node.cpp
)
ament_target_dependencies(leg_odometry_node
  rclcpp sensor_msgs nav_msgs geometry_msgs tf2 tf2_ros g1_msgs
)
target_link_libraries(leg_odometry_node
  Eigen3::Eigen
)

list(APPEND G1_SENSOR_BRIDGE_TARGETS unitree_bridge_node leg_odometry_node)
install(TARGETS
  ${G1_SENSOR_BRIDGE_TARGETS}
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
