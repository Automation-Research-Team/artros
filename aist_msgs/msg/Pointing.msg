int32 NO_RES=0
int32 SWEEP_RES=1
int32 RECAPTURE_RES=2

Header header                     # should give frame_id with respect to which finger_pos and finger_dir are represented
int32 pointing_state              # NO_RES: no responses to ROS, SWEEP_RES: finger_pos and finger_dir specifies sweep direction, RECAPTURE_RES: force ROS to recapture 3D point cloud
geometry_msgs/Point finger_pos    # finger position, i.e. right controller
geometry_msgs/Vector3 finger_dir  # finger direction, i.e. direction of beam from right controller
