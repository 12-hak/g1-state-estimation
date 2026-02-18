std_msgs/Header header

# Command type: 0=STOP, 1=GO_TO_POSE, 2=FOLLOW_WAYPOINTS, 3=CANCEL
uint8 command_type

uint8 STOP=0
uint8 GO_TO_POSE=1
uint8 FOLLOW_WAYPOINTS=2
uint8 CANCEL=3

# Target pose (for GO_TO_POSE)
geometry_msgs/PoseStamped goal

# Waypoints (for FOLLOW_WAYPOINTS)
geometry_msgs/PoseStamped[] waypoints
