int32 NO_RES=0
int32 SWEEP_RES=1
int32 RECAPTURE_RES=2

Header header
int32 pointing_state       # NO_RES: no responses to ROS, SWEEP_RES: specify sweep endpoint, RECAPTURE_RES: force ROS to recapture 3D point cloud
geometry_msgs/Point point  # sweep endpoint
