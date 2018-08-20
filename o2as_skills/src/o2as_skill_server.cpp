#include "o2as_skill_server.h"

SkillServer::SkillServer() : 
                  alignActionServer_(n_, "o2as_skills/align", boost::bind(&SkillServer::executeAlign, this, _1),false),
                  pickActionServer_(n_, "o2as_skills/pick", boost::bind(&SkillServer::executePick, this, _1),false),
                  placeActionServer_(n_, "o2as_skills/place", boost::bind(&SkillServer::executePlace, this, _1),false),
                  regraspActionServer_(n_, "o2as_skills/regrasp", boost::bind(&SkillServer::executeRegrasp, this, _1),false),
                  insertActionServer_(n_, "o2as_skills/insert", boost::bind(&SkillServer::executeInsert, this, _1),false),
                  screwActionServer_(n_, "o2as_skills/screw", boost::bind(&SkillServer::executeScrew, this, _1),false),
                  a_bot_group_("a_bot"), b_bot_group_("b_bot"), c_bot_group_("c_bot"),
                  b_bot_gripper_client_("/b_bot_gripper/gripper_action_controller", true),
                  c_bot_gripper_client_("/c_bot_gripper/gripper_action_controller", true)
{ 
  // Topics to publish
  pubMarker_ = n_.advertise<visualization_msgs::Marker>("visualization_marker", 10);
  visual_tools_.reset(new rviz_visual_tools::RvizVisualTools("world","viz_tools_marker"));
  ros::Duration(.5).sleep();

  // Services to advertise
  goToNamedPoseService_ = n_.advertiseService("o2as_skills/goToNamedPose", &SkillServer::goToNamedPoseCallback,
                                        this);

  // Services to subscribe to
  sendScriptToURClient_ = n_.serviceClient<o2as_msgs::sendScriptToUR>("o2as_skills/sendScriptToUR");
  PrecisionGripperClient_ = n_.serviceClient<o2as_msgs::PrecisionGripperCommand>("precision_gripper_command");

  // Actions we serve
  alignActionServer_.start();
  pickActionServer_.start();
  placeActionServer_.start();
  regraspActionServer_.start();
  insertActionServer_.start();
  screwActionServer_.start();

  // Action clients
  // ROS_INFO("Waiting for action servers to start.");
  // // a_bot_gripper_client.waitForServer(); 
  // b_bot_gripper_client_.waitForServer();
  // c_bot_gripper_client_.waitForServer();
  // ROS_INFO("Action servers started.");

  // Set up MoveGroups
  a_bot_group_.setPlanningTime(PLANNING_TIME);
  a_bot_group_.setPlannerId("RRTConnectkConfigDefault");
  a_bot_group_.setEndEffectorLink("a_bot_robotiq_85_tip_link");
  a_bot_group_.setNumPlanningAttempts(10);
  b_bot_group_.setPlanningTime(PLANNING_TIME);
  b_bot_group_.setPlannerId("RRTConnectkConfigDefault");
  b_bot_group_.setEndEffectorLink("b_bot_robotiq_85_tip_link");
  b_bot_group_.setNumPlanningAttempts(10);
  c_bot_group_.setPlanningTime(PLANNING_TIME);
  c_bot_group_.setPlannerId("RRTConnectkConfigDefault");
  c_bot_group_.setEndEffectorLink("c_bot_robotiq_85_tip_link");
  c_bot_group_.setNumPlanningAttempts(10);
  // front_bots_group_.setPlanningTime(PLANNING_TIME);
  // front_bots_group_.setPlannerId("RRTConnectkConfigDefault");
  // all_bots_group_.setPlanningTime(PLANNING_TIME);
  // all_bots_group_.setPlannerId("RRTConnectkConfigDefault");

  get_planning_scene_client = n_.serviceClient<moveit_msgs::GetPlanningScene>("/get_planning_scene");

  // Get the planning scene of the movegroup
  updatePlanningScene();

  // --- Define the screw tools.
  
  // Define one of the tools as a collision object
  screw_tool_m5.header.frame_id = "screw_tool_m5_link";
  screw_tool_m5.id = "screw_tool_m5";

  screw_tool_m5.primitives.resize(3);
  screw_tool_m5.primitive_poses.resize(3);
  // The bit cushion and motor
  screw_tool_m5.primitives[0].type = screw_tool_m5.primitives[0].BOX;
  screw_tool_m5.primitives[0].dimensions.resize(3);
  screw_tool_m5.primitives[0].dimensions[0] = 0.026;
  screw_tool_m5.primitives[0].dimensions[1] = 0.04;
  screw_tool_m5.primitives[0].dimensions[2] = 0.055;
  screw_tool_m5.primitive_poses[0].position.x = 0;
  screw_tool_m5.primitive_poses[0].position.y = -0.009;
  screw_tool_m5.primitive_poses[0].position.z = 0.0275;

  // The "shaft" + suction attachment
  screw_tool_m5.primitives[1].type = screw_tool_m5.primitives[1].BOX;
  screw_tool_m5.primitives[1].dimensions.resize(3);
  screw_tool_m5.primitives[1].dimensions[0] = 0.02;
  screw_tool_m5.primitives[1].dimensions[1] = 0.028;
  screw_tool_m5.primitives[1].dimensions[2] = 0.08;
  screw_tool_m5.primitive_poses[1].position.x = 0;
  screw_tool_m5.primitive_poses[1].position.y = -0.0055;  // 21 mm distance from axis
  screw_tool_m5.primitive_poses[1].position.z = -0.04;

  // The cylinder representing the tip
  screw_tool_m5.primitives[2].type = screw_tool_m5.primitives[2].CYLINDER;
  screw_tool_m5.primitives[2].dimensions.resize(2);
  screw_tool_m5.primitives[2].dimensions[0] = 0.02;    // Cylinder height
  screw_tool_m5.primitives[2].dimensions[1] = 0.0035;   // Cylinder radius
  screw_tool_m5.primitive_poses[2].position.x = 0;
  screw_tool_m5.primitive_poses[2].position.y = 0;  // 21 mm distance from axis
  screw_tool_m5.primitive_poses[2].position.z = -0.09;

  screw_tool_m5.operation = screw_tool_m5.ADD;
  
  // The other tools
  screw_tool_m4 = screw_tool_m5;
  screw_tool_m3 = screw_tool_m5;
}


// This works only for a single robot.
bool SkillServer::moveToCartPosePTP(geometry_msgs::PoseStamped pose, std::string robot_name, bool wait, std::string end_effector_link, double velocity_scaling_factor)
{
  moveit::planning_interface::MoveGroupInterface::Plan myplan;
  moveit::planning_interface::MoveItErrorCode 
    success_plan = moveit_msgs::MoveItErrorCodes::FAILURE, 
    motion_done = moveit_msgs::MoveItErrorCodes::FAILURE;

  moveit::planning_interface::MoveGroupInterface* group_pointer;
  group_pointer = robotNameToMoveGroup(robot_name);
  
  group_pointer->clearPoseTargets();
  group_pointer->setStartStateToCurrentState();
  group_pointer->setMaxVelocityScalingFactor(velocity_scaling_factor);  // TODO: Check if this works
  if (end_effector_link == "") group_pointer->setEndEffectorLink(robot_name + "_robotiq_85_tip_link"); // Force default
  else group_pointer->setEndEffectorLink(end_effector_link);
  group_pointer->setPoseTarget(pose);

  ROS_DEBUG_STREAM("Planning motion for robot " << robot_name << " and EE link " << end_effector_link + "_tip_link.");
  success_plan = group_pointer->plan(myplan);
  if (success_plan) 
  {
    if (wait) motion_done = group_pointer->execute(myplan);
    else motion_done = group_pointer->asyncExecute(myplan);
    if (motion_done) {
      group_pointer->setMaxVelocityScalingFactor(1.0); // Reset the velocity
      return true;
    }
  }
  ROS_WARN("Failed to perform motion.");
  group_pointer->setMaxVelocityScalingFactor(1.0); // Reset the velocity
  return false;
}
// This works only for a single robot.
bool SkillServer::moveToCartPoseLIN(geometry_msgs::PoseStamped pose, std::string robot_name, bool wait, std::string end_effector_link, double velocity_scaling_factor)
{
  moveit::planning_interface::MoveGroupInterface::Plan myplan;
  moveit::planning_interface::MoveItErrorCode 
    success_plan = moveit_msgs::MoveItErrorCodes::FAILURE, 
    motion_done = moveit_msgs::MoveItErrorCodes::FAILURE;

  moveit::planning_interface::MoveGroupInterface* group_pointer;
  group_pointer = robotNameToMoveGroup(robot_name);

  group_pointer->clearPoseTargets();
  group_pointer->setStartStateToCurrentState();
  group_pointer->setEndEffectorLink(end_effector_link);
  
  // Plan cartesian motion
  std::vector<geometry_msgs::Pose> waypoints;
  geometry_msgs::PoseStamped start_pose;
  if (end_effector_link == "") start_pose.header.frame_id = robot_name + "_robotiq_85_tip_link";
  else start_pose.header.frame_id = end_effector_link;
  start_pose.pose = makePose();
  start_pose = transform_pose_now(start_pose, "world", tflistener_);
  pose = transform_pose_now(pose, "world", tflistener_);
  waypoints.push_back(start_pose.pose);
  waypoints.push_back(pose.pose);

  ROS_WARN("Speed scaling does not work for linear motions. Going at regular speed.");
  group_pointer->setMaxVelocityScalingFactor(velocity_scaling_factor); // This doesn't work for linear paths: https://answers.ros.org/question/288989/moveit-velocity-scaling-for-cartesian-path/
  b_bot_group_.setPlanningTime(LIN_PLANNING_TIME);

  moveit_msgs::RobotTrajectory trajectory;
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;
  ros::Time start_time = ros::Time::now();
  double cartesian_success = group_pointer->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
  ros::Duration d = ros::Time::now() - start_time;
  ROS_INFO_STREAM("Cartesian motion plan took " << d.toSec() << " s and was " << cartesian_success * 100.0 << "% successful.");

  myplan.trajectory_ = trajectory;
  // if (cartesian_success > .95) 
  if (true) 
  {
    if (wait) motion_done = group_pointer->execute(myplan);
    else motion_done = group_pointer->asyncExecute(myplan);
    if (motion_done) 
    {
      group_pointer->setMaxVelocityScalingFactor(1.0); // Reset the velocity
      b_bot_group_.setPlanningTime(1.0);
      if (cartesian_success > .95) return true;
      else return false;
    }
  }
  group_pointer->setMaxVelocityScalingFactor(1.0); // Reset the velocity
  b_bot_group_.setPlanningTime(1.0);
  return false;
}

bool SkillServer::goToNamedPose(std::string pose_name, std::string robot_name)
{
  ROS_INFO_STREAM("Going to named pose " << pose_name << " with robot group " << robot_name << ".");
  // TODO: Test this.
  moveit::planning_interface::MoveGroupInterface* group_pointer;
  group_pointer = robotNameToMoveGroup(robot_name);

  group_pointer->setStartStateToCurrentState();
  group_pointer->setNamedTarget(pose_name);

  moveit::planning_interface::MoveGroupInterface::Plan myplan;
  moveit::planning_interface::MoveItErrorCode 
    success_plan = moveit_msgs::MoveItErrorCodes::FAILURE, 
    motion_done = moveit_msgs::MoveItErrorCodes::FAILURE;
  
  success_plan = group_pointer->move();
  
  return true;
}

// TODO: Write this function/decide if it is needed
bool SkillServer::stop()
{
  return true;
}

moveit::planning_interface::MoveGroupInterface* SkillServer::robotNameToMoveGroup(std::string robot_name)
{
  // This function converts the name of the robot to a pointer to the member variable containing the move group
  // Returning the move group itself does not seem to work, sadly.
  if (robot_name == "a_bot") return &a_bot_group_;
  if (robot_name == "b_bot") return &b_bot_group_;
  if (robot_name == "c_bot") return &c_bot_group_;
  // if (robot_name == "front_bots") return &front_bots_group_;
  // if (robot_name == "all_bots") return &all_bots_group_;
}


// ----------- Internal functions

bool SkillServer::equipScrewTool(std::string robot_name, std::string screw_tool_id)
{
  ROS_INFO_STREAM("Equipping screw tool " << screw_tool_id);
  return equipUnequipScrewTool(robot_name, screw_tool_id, "equip");
}

bool SkillServer::unequipScrewTool(std::string robot_name)
{
  ROS_INFO_STREAM("Unequipping screw tool " << held_screw_tool_);
  return equipUnequipScrewTool(robot_name, held_screw_tool_, "unequip");
}

bool SkillServer::equipUnequipScrewTool(std::string robot_name, std::string screw_tool_id, std::string equip_or_unequip)
{
  // Sanity check on the input instruction
  bool equip = (equip_or_unequip == "equip");
  bool unequip = (equip_or_unequip == "unequip");   
  // The second comparison is not always necessary, but readability comes first.
  if ((!equip) && (!unequip))
  {
    ROS_ERROR_STREAM("Cannot read the instruction " << equip_or_unequip << ". Returning false.");
    return false;
  }

  // TODO: Use the status of the robot_name instead of a general one
  if (holding_object_)
  {
    ROS_ERROR_STREAM("Robot already holds an object. Cannot " << equip_or_unequip << " screw tool.");
    return false;
  }
  
  // Plan & execute motion to in front of holder
  geometry_msgs::PoseStamped ps_approach, ps_tool_holder;
  ps_approach.header.frame_id = screw_tool_id + "_helper_link";

  ps_approach.pose.position.x = .0;   // If this is too far in the back, the LIN movement fails.
  ps_approach.pose.position.z = .017;
  ps_approach.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, 0);
  ROS_INFO("Moving to screw tool approach pose PTP.");
  moveToCartPosePTP(ps_approach, robot_name);

  if (equip) {
    openGripper(robot_name);
    ROS_INFO("Spawning tool.");
    spawnTool(screw_tool_id);
    held_screw_tool_ = screw_tool_id;
  }

  // Disable all collisions to allow movement into the tool
  // NOTE: This could be done cleaner by disabling only gripper + tool, but it is good enough for now.
  updatePlanningScene();
  ROS_INFO("Updating collision matrix.");
  collision_detection::AllowedCollisionMatrix acm_no_collisions(planning_scene_.allowed_collision_matrix),
                                              acm_original(planning_scene_.allowed_collision_matrix);
  acm_no_collisions.setEntry(screw_tool_id, true);   // Allow collisions with screw tool during pickup,
  acm_original.setEntry(screw_tool_id, false); // but not afterwards.
  std::vector<std::string> entries;
  acm_no_collisions.getAllEntryNames(entries);
  for (auto i : entries)
  {
    acm_no_collisions.setEntry(i, true);
  }
  moveit_msgs::PlanningScene ps_no_collisions = planning_scene_;
  acm_no_collisions.getMessage(ps_no_collisions.allowed_collision_matrix);
  planning_scene_interface_.applyPlanningScene(ps_no_collisions);

  // Plan & execute linear motion to the tool change position
  ps_tool_holder = ps_approach;
  if (equip)        ps_tool_holder.pose.position.x = 0.03;
  else if (unequip) ps_tool_holder.pose.position.x = 0.02;  

  // The tool is deposited a bit in front of the original position. The magnet pulls it to the final pose.
  ROS_INFO("Moving to pose in tool holder LIN.");
  moveToCartPoseLIN(ps_tool_holder, robot_name);

  // Close gripper, attach the tool object to the gripper in the Planning Scene.
  // Its collision with the parent link is set to allowed in the original planning scene.
  if (equip)
  {
    closeGripper(robot_name);
    attachTool(screw_tool_id, robot_name);
    acm_original.setEntry(screw_tool_id, robot_name + "_robotiq_85_tip_link", true);
  }
  else if (unequip) 
  {
    openGripper(robot_name);
    detachTool(screw_tool_id, robot_name);
    held_screw_tool_ = "";
    acm_original.removeEntry(screw_tool_id);
  }
  acm_original.getMessage(planning_scene_.allowed_collision_matrix);
  
  // Plan & execute LINEAR motion away from the tool change position
  ROS_INFO("Moving back to screw tool approach pose LIN.");
  moveToCartPoseLIN(ps_approach, robot_name);

  // Reactivate the collisions, with the updated entry about the tool
  planning_scene_interface_.applyPlanningScene(planning_scene_);

  // Delete tool collision object only after collision reinitialization to avoid errors
  if (unequip) despawnTool(screw_tool_id);

  return true;
}

// This refreshes the class member variable of the planning scene.
bool SkillServer::updatePlanningScene()
{
  moveit_msgs::GetPlanningScene srv;
  // Request only the collision matrix
  srv.request.components.components = moveit_msgs::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX;
  get_planning_scene_client.call(srv);
  if (get_planning_scene_client.call(srv))
  {
    ROS_INFO("Got planning scene from move group.");
    planning_scene_ = srv.response.scene;
    return true;
  }
  else
  {
    ROS_ERROR("Failed to get planning scene from move group.");
    return false;
  }
}

bool SkillServer::openGripper(std::string robot_name, std::string gripper_name)
{
  return sendGripperCommand(robot_name, 0.085, gripper_name);
}

bool SkillServer::closeGripper(std::string robot_name, std::string gripper_name)
{
  return sendGripperCommand(robot_name, 0.0, gripper_name);
}

bool SkillServer::sendGripperCommand(std::string robot_name, double opening_width, std::string gripper_name)
{
  bool finished_before_timeout;
  ROS_INFO_STREAM("Sending command to gripper of: " << robot_name);
  if ((robot_name == "a_bot"))
  {
    ROS_INFO_STREAM("Gripper: " << gripper_name);
    o2as_msgs::PrecisionGripperCommand srv;
    
    if (gripper_name == "")
    {
      ROS_WARN("No gripper was defined for a_bot! Using outer_gripper by default.");
      gripper_name = "outer_gripper";
    }
    
    if (gripper_name == "outer_gripper")
    {
      if (opening_width < 0.01) {srv.request.close_outer_gripper_fully = true;}
      else if (opening_width > 0.05) {srv.request.open_outer_gripper_fully = true;}
    }
    else if (gripper_name == "inner_gripper")
    {
      if (opening_width < 0.01) {srv.request.close_inner_gripper_fully = true;}
      else if (opening_width > 0.05) {srv.request.open_inner_gripper_fully = true;}
    }

    PrecisionGripperClient_.call(srv);
    if (srv.response.success == true)
    {
      ROS_INFO("Successfully sent the precision gripper command.");
      // srv.request.stop = true;
      // PrecisionGripperClient_.call(srv);
    }
    else
      ROS_ERROR("Could not send the precision gripper command.");
  }
  else if ((robot_name == "b_bot") || (robot_name == "c_bot"))
  {
    // Send a goal to the action
    robotiq_msgs::CModelCommandGoal goal;
    goal.position = opening_width;    // Opening width. 0 to close, 0.085 to open the gripper.
    if (robot_name == "b_bot")
    {
      b_bot_gripper_client_.sendGoal(goal);
      finished_before_timeout = b_bot_gripper_client_.waitForResult(ros::Duration(2.0));
    }
    else if (robot_name == "c_bot")
    {
      c_bot_gripper_client_.sendGoal(goal);
      finished_before_timeout = c_bot_gripper_client_.waitForResult(ros::Duration(2.0));
    }
  }
  else
  {
    ROS_ERROR("The specified gripper is not defined!");
    return false;
  }
  ROS_INFO("Returning from gripper command.");
  return finished_before_timeout;
}

// Add the screw tool as a Collision Object to the scene, so that it can be attached to the robot
bool SkillServer::spawnTool(std::string screw_tool_id)
{
  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.resize(1);

  if (screw_tool_id == "screw_tool_m5") collision_objects[0] = screw_tool_m5;
  else if (screw_tool_id == "screw_tool_m4") collision_objects[0] = screw_tool_m4;
  else if (screw_tool_id == "screw_tool_m3") collision_objects[0] = screw_tool_m3;
  
  collision_objects[0].operation = collision_objects[0].ADD;

  planning_scene_interface_.applyCollisionObjects(collision_objects);

  return true;
}

// Remove the tool from the scene so it does not cause unnecessary collision calculations
bool SkillServer::despawnTool(std::string screw_tool_id)
{
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.resize(1);

    collision_objects[0].id = screw_tool_id;
    collision_objects[0].operation = collision_objects[0].REMOVE;
    planning_scene_interface_.applyCollisionObjects(collision_objects);

    return true;
}

bool SkillServer::attachTool(std::string screw_tool_id, std::string robot_name)
{
  return attachDetachTool(screw_tool_id, robot_name, "attach");
}

bool SkillServer::detachTool(std::string screw_tool_id, std::string robot_name)
{
  return attachDetachTool(screw_tool_id, robot_name, "detach");
}

bool SkillServer::attachDetachTool(std::string screw_tool_id, std::string robot_name, std::string attach_or_detach)
{
  moveit_msgs::AttachedCollisionObject att_coll_object;

  if (screw_tool_id == "screw_tool_m5") att_coll_object.object = screw_tool_m5;
  else if (screw_tool_id == "screw_tool_m4") att_coll_object.object = screw_tool_m4;
  else if (screw_tool_id == "screw_tool_m3") att_coll_object.object = screw_tool_m3;
  else { ROS_WARN_STREAM("No screw tool specified to " << attach_or_detach); }

  att_coll_object.link_name = robot_name + "_robotiq_85_tip_link";

  if (attach_or_detach == "attach") att_coll_object.object.operation = att_coll_object.object.ADD;
  else if (attach_or_detach == "detach") att_coll_object.object.operation = att_coll_object.object.REMOVE;
  
  ROS_INFO_STREAM(attach_or_detach << "ing tool " << screw_tool_id);
  planning_scene_interface_.applyAttachedCollisionObject(att_coll_object);
  return true;
}

bool SkillServer::goFromAbove(geometry_msgs::PoseStamped target_tip_link_pose, std::string end_effector_link_name, std::string robot_name, double velocity_scaling_factor)
{
  ROS_INFO_STREAM("Received goFromAbove command.");

  // Move above the target pose
  target_tip_link_pose.pose.position.z += .1;
  ROS_INFO_STREAM("Moving above target.");
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  moveToCartPosePTP(target_tip_link_pose, robot_name, true, end_effector_link_name);

  // Move down to the target pose
  target_tip_link_pose.pose.position.z -= .1;
  ROS_INFO_STREAM("Moving down to target.");
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  bool success = moveToCartPoseLIN(target_tip_link_pose, robot_name, true, end_effector_link_name, velocity_scaling_factor);
  if (!success) 
  {
    ROS_INFO_STREAM("Linear motion plan to target place pose failed. Performing PTP.");
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    success = moveToCartPosePTP(target_tip_link_pose, robot_name, true, end_effector_link_name, velocity_scaling_factor);  // Force the move even if LIN fails
  }
  return success;
}

bool SkillServer::placeFromAbove(geometry_msgs::PoseStamped target_tip_link_pose, std::string end_effector_link_name, std::string robot_name, std::string gripper_name)
{
  publishMarker(target_tip_link_pose, "place_pose");
  ROS_INFO_STREAM("Received placeFromAbove command.");

  // Move above the target pose
  target_tip_link_pose.pose.position.z += .1;
  ROS_INFO_STREAM("Moving above object target place.");
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  moveToCartPosePTP(target_tip_link_pose, robot_name, true, end_effector_link_name);

  // Move down to the target pose
  target_tip_link_pose.pose.position.z -= .1;
  ROS_INFO_STREAM("Moving down to place object.");
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  bool success = 0;
  // bool success = moveToCartPoseLIN(target_tip_link_pose, robot_name, true, end_effector_link_name, 0.1);
  // if (!success) 
  // {
    // ROS_INFO_STREAM("Linear motion plan to target place pose failed. Performing PTP.");
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    moveToCartPosePTP(target_tip_link_pose, robot_name, true, end_effector_link_name, 0.01);  // Force the move even if LIN fails
  // }
  openGripper(robot_name, gripper_name);
  
  // // Move back up a little
  // target_tip_link_pose.pose.position.z += .05;
  // ROS_INFO_STREAM("Moving back up after placing object.");
  // std::this_thread::sleep_for(std::chrono::milliseconds(500));
  // success = moveToCartPoseLIN(target_tip_link_pose, robot_name, true, end_effector_link_name);
  // if (!success) 
  // {
  //   ROS_INFO_STREAM("Linear motion plan back from place pose failed. Performing PTP.");
  //   std::this_thread::sleep_for(std::chrono::milliseconds(500));
  //   moveToCartPosePTP(target_tip_link_pose, robot_name, true, end_effector_link_name);  // Force the move even if LIN fails
  // }

  ROS_INFO_STREAM("Finished placing object.");
  return true;
}

bool SkillServer::pickFromAbove(geometry_msgs::PoseStamped target_tip_link_pose, std::string end_effector_link_name, std::string robot_name, std::string gripper_name)
{
  publishMarker(target_tip_link_pose, "pick_pose");
  ROS_INFO_STREAM("Received pickFromAbove command.");
  

  // Move above the object
  openGripper(robot_name, gripper_name);
  target_tip_link_pose.pose.position.z += .1;
  ROS_INFO_STREAM("Opening gripper, moving above object.");
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  moveToCartPosePTP(target_tip_link_pose, robot_name, true, end_effector_link_name);

  // Move onto the object
  target_tip_link_pose.pose.position.z -= .1;
  ROS_INFO_STREAM("Moving down to pick object.");
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  bool success = moveToCartPoseLIN(target_tip_link_pose, robot_name, true, end_effector_link_name, 0.1);
  if (!success) 
  {
    ROS_INFO_STREAM("Linear motion plan to target pick pose failed. Performing PTP.");
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    moveToCartPosePTP(target_tip_link_pose, robot_name, true, end_effector_link_name, 0.1);  // Force the move even if LIN fails
  }
  closeGripper(robot_name, gripper_name);

  // Move back up a little
  target_tip_link_pose.pose.position.z += .1;
  ROS_INFO_STREAM("Moving back up after picking object.");
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  success = moveToCartPoseLIN(target_tip_link_pose, robot_name, true, end_effector_link_name);
  if (!success) 
  {
    ROS_INFO_STREAM("Linear motion plan back from pick pose failed. Performing PTP.");
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    moveToCartPosePTP(target_tip_link_pose, robot_name, true, end_effector_link_name);  // Force the move even if LIN fails
  }

  ROS_INFO_STREAM("Finished picking object.");
  return true;
}

bool SkillServer::pickScrew(std::string object_id, std::string screw_tool_id, std::string robot_name)
{
  // Retrieve the position of the object from the planning scene
  geometry_msgs::PoseStamped object_pose = makePoseStamped();
  std::vector<std::string> object_name_vector;
  object_name_vector.push_back(object_id);
  std::map< std::string, geometry_msgs::Pose > poses = planning_scene_interface_.getObjectPoses(object_name_vector);
  
  object_pose.header.frame_id = "world";
  object_pose.pose.position = poses[object_id].position;
  // We ignore the orientation of the screw (they always point down) and assign an appropriate orientation
  // TODO: How to find a good pose for the robot automatically? Launch a planning request to MoveIt?
  object_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, -(145.0/180.0 *M_PI));   // Z-axis pointing up.
  
  std::string screw_tool_link = robot_name + "_" + screw_tool_id + "_tip_link";
  return pickFromAbove(object_pose, screw_tool_link, robot_name);
}

bool SkillServer::publishMarker(geometry_msgs::PoseStamped marker_pose, std::string marker_type)
{

  visualization_msgs::Marker marker;
  marker.header = marker_pose.header;
  marker.header.stamp = ros::Time::now();
  marker.pose = marker_pose.pose;

  marker.ns = "markers";
  marker.id = marker_id_count++;
  marker.lifetime = ros::Duration();
  marker.action = visualization_msgs::Marker::ADD;

  if (marker_type == "pose")
  {
    publishPoseMarker(marker_pose);

    // Add a flat sphere
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.scale.x = .01;
    marker.scale.y = .05;
    marker.scale.z = .05;
    marker.color.g = 1.0;
    marker.color.a = 0.8;
    pubMarker_.publish(marker);
    return true;
  }
  if (marker_type == "place_pose")
  {
    publishPoseMarker(marker_pose);

    // Add a flat sphere
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.scale.x = .01;
    marker.scale.y = .05;
    marker.scale.z = .05;
    marker.color.g = 1.0;
    marker.color.a = 0.8;
    pubMarker_.publish(marker);
    return true;
  }
  if (marker_type == "pick_pose")
  {
    publishPoseMarker(marker_pose);

    // Add a flat sphere
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.scale.x = .01;
    marker.scale.y = .05;
    marker.scale.z = .05;
    marker.color.r = 0.8;
    marker.color.g = 0.4;
    marker.color.a = 0.8;
  }
  else if (marker_type == "")
  {
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.scale.x = .02;
    marker.scale.y = .1;
    marker.scale.z = .1;
    
    marker.color.g = 1.0;
    marker.color.a = 0.8;
  }
  else 
  {ROS_WARN("No useful marker message received.");}
  pubMarker_.publish(marker);
  if (marker_id_count > 50) marker_id_count = 0;
  return true;
}

// This is a helper function for publishMarker. Publishes a TF-like frame.
bool SkillServer::publishPoseMarker(geometry_msgs::PoseStamped marker_pose)
{
  geometry_msgs::PoseStamped ps = transform_pose_now(marker_pose, "world", tflistener_);
  visual_tools_->publishAxis(ps.pose);
  visual_tools_->trigger();
  return true;
}
// ----------- Service definitions
bool SkillServer::goToNamedPoseCallback(o2as_msgs::goToNamedPose::Request &req,
                                           o2as_msgs::goToNamedPose::Response &res)
{
  ROS_INFO("Received goToNamedPose callback.");
  res.success = goToNamedPose(req.named_pose, req.planning_group);
  return true;
}


// ----------- Action servers

// alignAction
void SkillServer::executeAlign(const o2as_msgs::alignGoalConstPtr& goal)
{
  ROS_INFO("alignAction was called");
  // TODO: Insert commands to do the action
  std::this_thread::sleep_for(std::chrono::milliseconds(2000));
  ROS_INFO("alignAction is set as succeeded");
  alignActionServer_.setSucceeded();
}

// pickAction
void SkillServer::executePick(const o2as_msgs::pickGoalConstPtr& goal)
{
  ROS_INFO("pickAction was called");

  if ((goal->gripper_command == "complex_pick_from_inside") || (goal->gripper_command == "complex_pick_from_outside"))
  {
    geometry_msgs::PoseStamped target_tip_link_pose = goal->item_pose;
    std::string end_effector_link_name = goal->robot_name + "_gripper_tip_link";
    // Do the positioning with the inner gripper first, then close the outer gripper
    publishMarker(target_tip_link_pose, "pick_pose");
    ROS_INFO_STREAM("Doing complex pick with custom gripper.");
    
    // Move above the object
    openGripper("a_bot", "outer_gripper");
    if (goal->gripper_command == "complex_pick_from_inside")
    {
      ROS_INFO_STREAM("Opening outer and closing inner gripper, moving above object.");
      closeGripper("a_bot", "inner_gripper");
      std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    }
    else if (goal->gripper_command == "complex_pick_from_outside")
    {
      ROS_INFO_STREAM("Opening outer and inner gripper, moving above object.");
      openGripper("a_bot", "inner_gripper");
      std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    }
    target_tip_link_pose.pose.position.z += .1;
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    moveToCartPosePTP(target_tip_link_pose, goal->robot_name, true, end_effector_link_name);

    ROS_INFO_STREAM("Moving closer to object (5 cm offset).");
    target_tip_link_pose.pose.position.z -= .05;
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    bool success = moveToCartPoseLIN(target_tip_link_pose, goal->robot_name, true, end_effector_link_name, 0.1);
    if (!success) 
    {
      ROS_INFO_STREAM("Linear motion plan failed. Performing PTP.");
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
      moveToCartPosePTP(target_tip_link_pose, goal->robot_name, true, end_effector_link_name, 0.1);  // Force the move even if LIN fails
    }

    target_tip_link_pose.pose.position.z -= .05;
    ROS_INFO_STREAM("Moving down to object (no offset).");
    moveToCartPosePTP(target_tip_link_pose, goal->robot_name, true, end_effector_link_name, 0.01);  // Go very slow
    // TODO: Send the MoveL command to the real robot instead?
    
    if (goal->gripper_command == "complex_pick_from_inside")
    {
      ROS_INFO_STREAM("Opening inner gripper to position object.");
      openGripper("a_bot", "inner_gripper");
      std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    }
    else if (goal->gripper_command == "complex_pick_from_outside")
    {
      ROS_INFO_STREAM("Closing inner gripper to position object.");
      closeGripper("a_bot", "inner_gripper");
      std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    }
    closeGripper("a_bot", "outer_gripper");

    ROS_INFO_STREAM("Moving back up after picking object.");
    target_tip_link_pose.pose.position.z += .1;
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    success = moveToCartPoseLIN(target_tip_link_pose, goal->robot_name, true, end_effector_link_name);
    if (!success) 
    {
      ROS_INFO_STREAM("Linear motion plan back from pick pose failed. Performing PTP.");
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
      moveToCartPosePTP(target_tip_link_pose, goal->robot_name, true, end_effector_link_name);  // Force the move even if LIN fails
    }

        if (goal->gripper_command == "complex_pick_from_inside")
    {
      ROS_INFO_STREAM("Closing inner gripper again after outer gripper grasped object.");
      closeGripper("a_bot", "inner_gripper");
      std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    }
    else if (goal->gripper_command == "complex_pick_from_outside")
    {
      ROS_INFO_STREAM("Opening inner gripper again after outer gripper grasped object.");
      openGripper("a_bot", "inner_gripper");
      std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    }

    ROS_INFO_STREAM("Finished picking object.");
  }
  else if (goal->tool_name == "screw_tool")
  {
    std::string screw_tool_id = goal->tool_name + "_m" + std::to_string(goal->screw_size);
    pickScrew(goal->item_id, screw_tool_id, goal->robot_name);
  }
  else if (goal->tool_name == "suction")
  {;} // TODO: Here is space for code from AIST.
  else // No special tool being used; use just one gripper instead
  {
    // Warning: Using only "abs" rounds to int. Amazing.
    if ((std::abs(goal->item_pose.pose.position.x)) > 0 || (std::abs(goal->item_pose.pose.position.y)) > 0 || (std::abs(goal->item_pose.pose.position.z)) > 0)
    {
      std::string ee_link_name;
      if (goal->robot_name == "a_bot"){ee_link_name = goal->robot_name + "_gripper_tip_link"; }
      else {ee_link_name = goal->robot_name + "_robotiq_85_tip_link";}
      pickFromAbove(goal->item_pose, ee_link_name, goal->robot_name);
    }
    else
    {
      ROS_ERROR("Item_pose is empty and no tool_name was set. Not doing anything");
    }

    // TODO. The plan: 
    // - Check that the object exists in the planning scene
    // - VARIATION A:
    // -- Pass the request to the grasp planner along with the robot name
    // -- It should return a grasp to be executed via the modified MoveIt pick routine
    // - VARIATION B:
    // -- Take the center of gravity and the major axis of the object
    // -- Move the gripper to above the object, orienting the major axis along the z-axis of the tip link
    // -- Allow collisions with the object and the gripper
    // -- Move down to an appropriate gripping height or to the bottom of the container
    // -- Close the grasp, then move back up
  }

  ROS_INFO("pickAction is set as succeeded");
  pickActionServer_.setSucceeded();
}

// placeAction
void SkillServer::executePlace(const o2as_msgs::placeGoalConstPtr& goal)
{
  ROS_INFO("placeAction was called");
  // TODO: Calculate the target pose with the item height currently held

  std::string ee_link_name;
  if (goal->robot_name == "a_bot"){ee_link_name = goal->robot_name + "_gripper_tip_link"; }
  else {ee_link_name = goal->robot_name + "_robotiq_85_tip_link";}
  
  if (goal->tool_name == "suction")
  {
    ; // TODO: Set the ee_link_name correctly and pass a flag to placeFromAbove
  }

  placeFromAbove(goal->item_pose, ee_link_name, goal->robot_name);
  ROS_INFO("placeAction is set as succeeded");
  placeActionServer_.setSucceeded();
}

// regraspAction
void SkillServer::executeRegrasp(const o2as_msgs::regraspGoalConstPtr& goal)
{
  ROS_INFO("regraspAction was called");
  
  // Create the handover_pose for first robot and second robot
  // Start by making a few motion primitives.
  geometry_msgs::PoseStamped c_bot_facing_forward, a_bot_facing_forward, b_bot_facing_forward, a_bot_facing_c, b_bot_facing_c;
  c_bot_facing_forward.header.frame_id = "workspace_center";
  a_bot_facing_forward.header.frame_id = "workspace_center";
  b_bot_facing_forward.header.frame_id = "workspace_center";
  a_bot_facing_c.header.frame_id = "workspace_center";
  b_bot_facing_c.header.frame_id = "workspace_center";
  
  c_bot_facing_forward.pose.position.x = -.35;
  c_bot_facing_forward.pose.position.y = 0.11;
  c_bot_facing_forward.pose.position.z = 0.55;
  c_bot_facing_forward.pose.orientation.w = 1.0;

  b_bot_facing_forward.pose.position.x = 0.0;
  b_bot_facing_forward.pose.position.y = 0.1;
  b_bot_facing_forward.pose.position.z = 0.65;
  b_bot_facing_forward.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, -M_PI/2);

  a_bot_facing_forward.pose.position.x = 0.0;
  a_bot_facing_forward.pose.position.y = 0.1;
  a_bot_facing_forward.pose.position.z = 0.65;
  a_bot_facing_forward.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(-M_PI/2, 0, M_PI/2);

  a_bot_facing_c.pose.position.x = -.28;
  a_bot_facing_c.pose.position.y = 0.11;
  a_bot_facing_c.pose.position.z = 0.55;
  a_bot_facing_c.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, -M_PI);

  b_bot_facing_c.pose.position.x = -.28;
  b_bot_facing_c.pose.position.y = 0.11;
  b_bot_facing_c.pose.position.z = 0.55;
  a_bot_facing_c.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(-M_PI, 0, M_PI/2);


  geometry_msgs::PoseStamped handover_pose_giver, handover_pose_receiver;
  if (goal->giver_robot_name == "a_bot")
  {
    if (goal->receiver_robot_name == "c_bot")
    {
      handover_pose_giver = a_bot_facing_c;
      handover_pose_receiver = c_bot_facing_forward;
    }
    else if (goal->receiver_robot_name == "b_bot")
    {
      handover_pose_giver = a_bot_facing_forward;
      handover_pose_receiver = b_bot_facing_forward;
    }
  }
  else if (goal->giver_robot_name == "b_bot")
  {
    if (goal->receiver_robot_name == "c_bot")
    {
      handover_pose_giver = b_bot_facing_c;
      handover_pose_receiver = c_bot_facing_forward;
    }
    else if (goal->receiver_robot_name == "b_bot")
    {
      handover_pose_giver = a_bot_facing_forward;
      handover_pose_receiver = b_bot_facing_forward;
    }
  }
  else if (goal->giver_robot_name == "c_bot")
  {
    handover_pose_giver = c_bot_facing_forward;
    if (goal->receiver_robot_name == "a_bot")
    {
      handover_pose_receiver = a_bot_facing_c;
    }
    else if (goal->receiver_robot_name == "b_bot")
    {
      handover_pose_receiver = b_bot_facing_c;
    }
  }

  // Move the first robot to the regrasp_pose
  ROS_INFO_STREAM("Moving giver robot (" << goal->giver_robot_name << ") to handover pose.");
  moveToCartPosePTP(handover_pose_giver, goal->giver_robot_name);
  
  // Move the second robot to an approach pose, then on to grasp
  ROS_INFO_STREAM("Moving receiver robot (" << goal->giver_robot_name << ") to approach pose.");
  handover_pose_receiver.pose.position.x -= .1;
  moveToCartPosePTP(handover_pose_receiver, goal->receiver_robot_name);

  ros::Duration(1).sleep();
  ROS_INFO_STREAM("Moving receiver robot (" << goal->giver_robot_name << ") to grasp pose.");
  handover_pose_receiver.pose.position.x += .1;
  moveToCartPosePTP(handover_pose_receiver, goal->receiver_robot_name);
  
  // Close the second robot's gripper
  closeGripper(goal->receiver_robot_name);
  ros::Duration(1).sleep();
  openGripper(goal->giver_robot_name);

  // Move back.
  ROS_INFO_STREAM("Moving receiver robot (" << goal->giver_robot_name << ") back to approach pose.");
  handover_pose_receiver.pose.position.x -= .1;
  moveToCartPosePTP(handover_pose_receiver, goal->receiver_robot_name);

  // goToNamedPose("home", goal->giver_robot_name);

  ROS_INFO("regraspAction is set as succeeded");
  regraspActionServer_.setSucceeded();
}

// insertAction
void SkillServer::executeInsert(const o2as_msgs::insertGoalConstPtr& goal)
{
  ROS_INFO("insertAction was called");
  std::string ee_link_name;
  if (goal->robot_name == "a_bot"){ee_link_name = goal->robot_name + "_gripper_tip_link"; }
  else {ee_link_name = goal->robot_name + "_robotiq_85_tip_link";}

  // TODO: Set the height/offset!
  goFromAbove(goal->target_hole, ee_link_name, goal->robot_name, 0.1);
  
  o2as_msgs::sendScriptToUR srv;
  srv.request.program_id = "insertion";
  srv.request.robot_name = goal->robot_name;
  srv.request.stroke = goal->maximum_insertion_distance;
  srv.request.force_magnitude = goal->maximum_force;
  srv.request.forward_speed = goal->speed;
  sendScriptToURClient_.call(srv);
  if (srv.response.success == true)
    ROS_INFO("Successfully called the URScript client to start insertion");
  else
    ROS_WARN("Could not call the URScript client to start insertion");
  
  geometry_msgs::PoseStamped ps = goal->target_hole;
  

  ROS_INFO("insertAction is set as succeeded");
  insertActionServer_.setSucceeded();
}

// screwAction
void SkillServer::executeScrew(const o2as_msgs::screwGoalConstPtr& goal)
{
  ROS_INFO("screwAction was called");

  // Set target pose for the end effector
  geometry_msgs::PoseStamped target_tip_link_pose = goal->target_hole;
  std::string screw_tool_link = goal->robot_name + "_" + held_screw_tool_ + "_tip_link";

  target_tip_link_pose.pose.position.z += goal->screw_height+.005;  // Add the screw height and tolerance

  if (goal->screw_height < 0.001) {target_tip_link_pose.pose.position.z += .02;}   // In case screw_height was not set

  // Move above the screw hole
  target_tip_link_pose.pose.position.z += .05;
  moveToCartPosePTP(target_tip_link_pose, goal->robot_name, true, screw_tool_link);

  // Move down to the screw hole
  target_tip_link_pose.pose.position.z -= .05;
  bool success = moveToCartPoseLIN(target_tip_link_pose, goal->robot_name, true, screw_tool_link);
  if (!success) moveToCartPosePTP(target_tip_link_pose, goal->robot_name, true, screw_tool_link);  // Force the move even if LIN fails

  // TODO: Move down, exert very light pressure + spiral motion to find the screw hole
  //       This should be done via a URscript.
  //       Also turn on the fastener tool motor.
  
  // Move back up a little
  target_tip_link_pose.pose.position.z += .05;
  success = moveToCartPoseLIN(target_tip_link_pose, goal->robot_name, true, screw_tool_link);
  if (!success) moveToCartPosePTP(target_tip_link_pose, goal->robot_name, true, screw_tool_link);  // Force the move even if LIN fails
  
  ROS_INFO("screwAction is set as succeeded");
  screwActionServer_.setSucceeded();
}

// ----------- End of the class definitions

int main(int argc, char **argv)
{
  ros::init(argc, argv, "o2as_skills");
  ros::AsyncSpinner spinner(1); // Needed for MoveIt to work.
  spinner.start();

  // Create an object of class SkillServer that will take care of everything
  SkillServer o2as_skill_server;
  ROS_INFO("O2AS skill server started");


  // ############## Test markers
  geometry_msgs::PoseStamped ps1, ps2;
  ps1 = makePoseStamped();
  ps1.pose.position.z = .3;
  ps2 = ps1;
  ps1.pose.position.x = .5;

  while (ros::ok())
  {
    ros::spinOnce();
    o2as_skill_server.publishPoseMarker(ps1);
    o2as_skill_server.publishPoseMarker(ps2);
    ros::Duration(1).sleep();
    o2as_skill_server.visual_tools_->deleteAllMarkers();
  }
  // ###############

  // o2as_msgs::sendScriptToUR srv;
  // srv.request.program_id = "insertion";
  // srv.request.robot_name = "b_bot";
  // o2as_skill_server.sendScriptToURClient_.call(srv);
  // if (srv.response.success == true)
  //   ROS_INFO("Successfully called the service client");
  // else
  //   ROS_WARN("Could not call the service client");

  //// ------------ Debugging procedures. Should be in a separate node, but ohwell.
  // ROS_INFO("Testing the screw tool mounting.");
  // o2as_skill_server.equipScrewTool("b_bot", "screw_tool_m5");

  // ROS_INFO("Going to screw ready pose.");
  // o2as_skill_server.goToNamedPose("screw_ready_b", "b_bot");

  // ROS_INFO("Picking screw from feeder.");
  // geometry_msgs::PoseStamped ps = makePoseStamped();
  // ps.header.frame_id = "m5_feeder_outlet_link";
  // ps.pose.position.z = .01;
  // ps.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, -(110.0/180.0 *M_PI));   // Z-axis pointing up.
  // std::string link_name = "b_bot_screw_tool_m5_tip_link";
  // std::string robot_name = "b_bot";

  // o2as_skill_server.pickFromAbove(ps, link_name, robot_name);

  // ROS_INFO("Picking screw from tray 1.");
  // ps.header.frame_id = "set3_tray_1";
  // ps.pose.position.y = .05;
  // ps.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, -(90.0/180.0 *M_PI));   // Z-axis pointing up.
  // o2as_skill_server.pickFromAbove(ps, link_name, robot_name);

  // ROS_INFO("Going back to screw ready pose.");
  // o2as_skill_server.goToNamedPose("screw_ready_b", "b_bot");

  // ROS_INFO("Testing the screw tool unmounting.");
  // o2as_skill_server.unequipScrewTool("b_bot");

  // ROS_INFO("Going to home pose.");
  // o2as_skill_server.goToNamedPose("home_b", "b_bot");
  // ROS_INFO("Done.");
  //// ------------

  while (ros::ok())
  {
    ros::spinOnce();
  }

  return 0;
}
