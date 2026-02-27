cmake_minimum_required(VERSION 3.8)
project(g1_slam)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(pcl_conversions QUIET)
find_package(pcl_ros QUIET)
find_package(g1_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL QUIET COMPONENTS common io filters kdtree registration)

# Build SLAM nodes only when PCL and ROS PCL packages are available (e.g. after: sudo apt install ros-humble-pcl-ros ros-humble-pcl-conversions)
if(pcl_conversions_FOUND AND pcl_ros_FOUND AND PCL_FOUND)
  include_directories(
    include
    ${EIGEN3_INCLUDE_DIR}
    ${PCL_INCLUDE_DIRS}
  )
  link_directories(${PCL_LIBRARY_DIRS})
  add_definitions(${PCL_DEFINITIONS})

  # --- Core SLAM library ---
  add_library(g1_slam_core SHARED
    src/kiss_icp_wrapper.cpp
    src/ekf_fusion.cpp
    src/loop_closure.cpp
    src/pose_graph.cpp
    src/voxel_hash_map.cpp
  )
  target_link_libraries(g1_slam_core
    Eigen3::Eigen
    ${PCL_LIBRARIES}
  )

  # --- SLAM Node ---
  add_executable(slam_node src/slam_node.cpp)
  target_link_libraries(slam_node g1_slam_core)
  ament_target_dependencies(slam_node
    rclcpp sensor_msgs nav_msgs geometry_msgs tf2 tf2_ros tf2_eigen
    pcl_conversions pcl_ros g1_msgs
  )

  # --- Occupancy Grid Node ---
  add_executable(occupancy_grid_node src/occupancy_grid_node.cpp)
  target_link_libraries(occupancy_grid_node g1_slam_core)
  ament_target_dependencies(occupancy_grid_node
    rclcpp sensor_msgs nav_msgs geometry_msgs pcl_conversions pcl_ros
  )

  # --- Map Manager Node ---
  add_executable(map_manager_node src/map_manager_node.cpp)
  target_link_libraries(map_manager_node g1_slam_core)
  ament_target_dependencies(map_manager_node
    rclcpp sensor_msgs nav_msgs g1_msgs pcl_conversions pcl_ros
  )

  install(TARGETS
    slam_node
    occupancy_grid_node
    map_manager_node
    DESTINATION lib/${PROJECT_NAME}
  )

  install(TARGETS g1_slam_core
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
  )
else()
  message(STATUS "PCL/pcl_ros not found: g1_slam nodes will not be built. Install: sudo apt install ros-humble-pcl-ros ros-humble-pcl-conversions")
endif()

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
