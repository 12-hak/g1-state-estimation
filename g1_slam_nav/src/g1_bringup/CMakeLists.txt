cmake_minimum_required(VERSION 3.8)
project(g1_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config rviz urdf
  DESTINATION share/${PROJECT_NAME}
)
install(PROGRAMS scripts/publish_robot_description.py scripts/point_lio_odom_to_tf.py
  DESTINATION lib/${PROJECT_NAME}
)
install(PROGRAMS scripts/point_lio_odom_to_tf.py
  DESTINATION libexec/${PROJECT_NAME}
)

ament_package()
