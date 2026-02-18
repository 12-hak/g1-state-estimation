std_msgs/Header header

# Pose in map frame
geometry_msgs/Pose pose

# Velocity in base frame
geometry_msgs/Twist velocity

# Joint state summary
float32[] joint_positions
float32[] joint_velocities
float32[] joint_torques

# IMU
geometry_msgs/Quaternion imu_orientation
geometry_msgs/Vector3 imu_angular_velocity
geometry_msgs/Vector3 imu_linear_acceleration

# Status flags
bool is_stationary
bool icp_valid
float32 icp_error
float32 battery_voltage

# Robot type identifier (g1=1, go2=2)
uint8 robot_type
