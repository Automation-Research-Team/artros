aist_controllers
==================================================

## `pose_head_tracker`
### Parameters
 - **~controller**:
   - type=`string`
   - default=`/pos_joint_traj_controller`
   - その状態をsubscribeし，コマンドを送付する対象となるコントローラ名
 - **~robot_description**:
   - type=`string`
   - default=`/robot_description`　
   - ロボットの記述を格納したROSパラメータ名
 - **~base_link**:
   - type=`string`
   - default=`base_link`
   - アームのリンク機構の起点となるフレーム名
 - **~controller/state_publish_rate**:
   - type=`double`
   - default=`50`
 
### Action
 - **~pose_head**:
   - type=[aist_controllers/PoseHeadAction](action/PoseHead.action)

### Subscribed topic
 - **~controller/state**:
   - type=[control_msgs/JointTrajectoryControllerState](http://docs.ros.org/en/api/control_msgs/html/msg/JointTrajectoryControllerState.html)
 
### Published topic
 - **~controller/command**:
   - type=[trajectory_msgs/JointTrajectory](http://docs.ros.org/en/api/trajectory_msgs/html/msg/JointTrajectory.html)


## `pose_group_tracker`
### Parameters
 - **~controller**:
   - type=`string`
   - default=`/pos_joint_group_controller`
   - コマンドを送付する対象となるコントローラ名
 - **~robot_description**:
   - type=`string`
   - default=`/robot_description`　
   - ロボットの記述を格納したROSパラメータ名
 - **~base_link**:
   - type=`string`
   - default=`base_link`
   - アームのリンク機構の起点となるフレーム名
 - **~controller/state_publish_rate**:
   - type=`double`
   - default=`50`
 
### Action
 - **~pose_head**:
   - type=[aist_controllers/PoseHeadAction](action/PoseHead.action)
### Subscribed topic
 - **/joint_states**:
   - type=[sensor_msgs/JointState](http://docs.ros.org/en/api/sensor_msgs/html/msg/JointState.html)
 
### Published topic
 - **~controller/command**:
   - type=[std_msgs/Float64MultiArray](http://docs.ros.org/en/api/std_msgs/html/msg/Float64MultiArray.html)
   
