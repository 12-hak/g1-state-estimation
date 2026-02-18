std_msgs/Header header

# SLAM state
bool mapping_active
bool localization_valid
uint32 map_point_count
uint32 loop_closure_count
float32 odometry_quality

# Map bounds (meters, in map frame)
float32 map_min_x
float32 map_min_y
float32 map_max_x
float32 map_max_y

# Current map resolution
float32 map_resolution
