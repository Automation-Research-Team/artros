int32 NO_REQ=0
int32 SWEEP_DIR_REQ=1

string robot_name               # name of the robot, "a_bot" or "b_bot"
string item_id                  # ID of item to be picked (not used now)
geometry_msgs/PoseStamped pose  # position/orientation where error happened
int32 request                   # if SWEEP_DIR_REQ, request for sweep direction
string message                  # error message to the operator (not used now)
