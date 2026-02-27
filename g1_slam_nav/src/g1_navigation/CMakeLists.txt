cmake_minimum_required(VERSION 3.8)
project(g1_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs QUIET)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(g1_msgs REQUIRED)

# Navigation command server (only if nav2_msgs is available, e.g. after: sudo apt install ros-humble-nav2-msgs)
if(nav2_msgs_FOUND)
  add_executable(nav_command_node src/nav_command_node.cpp)
  ament_target_dependencies(nav_command_node
    rclcpp rclcpp_action nav_msgs geometry_msgs nav2_msgs tf2 tf2_ros g1_msgs
  )
  install(TARGETS
    nav_command_node
    DESTINATION lib/${PROJECT_NAME}
  )
else()
  message(STATUS "nav2_msgs not found: nav_command_node will not be built. Install ros-humble-nav2-msgs for full navigation.")
endif()

install(DIRECTORY launch params
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
