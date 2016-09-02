# Define possible Requests
uint8 PICK_REQUEST = 0
uint8 PLACE_REQUEST = 1
uint8 MOVETO_REQUEST = 2
uint8 PUSH_REQUEST = 3
uint8 HOLD_REQUEST = 4
uint8 RELEASE_REQUEST = 5

std_msgs/Header header

uint8 request

geometry_msgs/Pose object_pose
