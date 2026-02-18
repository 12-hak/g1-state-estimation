std_msgs/Header header

# Ordered list of waypoints in map frame
geometry_msgs/PoseStamped[] waypoints

# Per-waypoint tolerance radius (meters); empty = use default
float32[] tolerances

# If true, robot should return to first waypoint after completing the list
bool loop
