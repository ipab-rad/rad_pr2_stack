# Define possible Requests
uint8 PICK_REQUEST = 0
uint8 PLACE_REQUEST = 1
uint8 MOVETO_REQUEST = 2

std_msgs/Header header

uint8 request

geometry_msgs/Pose object_pose
