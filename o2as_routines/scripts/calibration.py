#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2013, SRI International
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of SRI International nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Felix von Drigalski

import sys
import copy
import rospy

import geometry_msgs.msg
import tf
import tf_conversions
from math import pi

from o2as_routines.base import O2ASBaseRoutines

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CalibrationClass(O2ASBaseRoutines):
  """
  These routines check the robots' calibration by moving them to
  objects defined in the scene.
  """

  def __init__(self):
    super(CalibrationClass, self).__init__()
    
    self.a_bot_downward_orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, pi))
    self.bin_names = ["bin2_1", "bin2_2", "bin2_3", "bin2_4", "bin3_1", "bin1_1", 
                      "bin1_2", "bin1_3", "bin1_4", "bin1_5"]

    self.bridge = CvBridge()
    self._img = Image()
    self.img_sub = rospy.Subscriber("/a_bot_camera/color/image_raw", Image, self.image_callback)

    # Neutral downward in the taskboard frames
    rospy.sleep(.5)   # Use this instead of waiting, so that simulation can be used

  def cycle_through_calibration_poses(self, poses, robot_name, speed=0.3, with_approach=False, move_lin=False, go_home=True, end_effector_link=""):
    home_pose = "home"
      
    # rospy.loginfo("============ Moving " + robot_name + " to " + poses[0].header.frame_id)
    if with_approach:                 # To calculate the approach, we publish the target pose to TF
      rospy.logwarn("with_approach does not work yet. Do not use it.")
      # ps_approach = geometry_msgs.msg.PoseStamped()
      # ps_approach.header.frame_id = "calibration_target_pose"
      # ps_approach.pose.position.x -= .05

    for pose in poses:  
      rospy.loginfo("============ Press `Enter` to move " + robot_name + " to " + pose.header.frame_id)
      self.publish_marker(pose, "place_pose")
      raw_input()
      if go_home:
        self.go_to_named_pose(home_pose, robot_name)
      if with_approach:
        ps_approach = copy.deepcopy(pose) # Dirty fix for the TF frame below not being found
        # br = tf.TransformBroadcaster()
        # br.sendTransform((pose.pose.position.x, pose.pose.position.y, pose.pose.position.z),
        #                   (pose.pose.orientation.x, pose.pose.orientation.y,
        #                    pose.pose.orientation.z, pose.pose.orientation.w), rospy.Time.now(),
        #                    "calibration_target_pose", pose.header.frame_id)
        # rospy.sleep(.5)
        self.go_to_pose_goal(robot_name, ps_approach,speed=speed,end_effector_link=end_effector_link, move_lin = move_lin)
      if rospy.is_shutdown():
        break
      if with_approach:
        self.go_to_pose_goal(robot_name, ps_approach,speed=speed,end_effector_link=end_effector_link, move_lin = move_lin)
        self.go_to_pose_goal(robot_name, pose,speed=speed,end_effector_link=end_effector_link, move_lin = move_lin)
      else:
        self.go_to_pose_goal(robot_name, pose,speed=speed,end_effector_link=end_effector_link, move_lin = move_lin)
      
      rospy.loginfo("============ Press `Enter` to proceed ")
      raw_input()
      if with_approach:
        # br = tf.TransformBroadcaster()
        # br.sendTransform((pose.pose.position.x, pose.pose.position.y, pose.pose.position.z),
        #                   (pose.pose.orientation.x, pose.pose.orientation.y,
        #                    pose.pose.orientation.z, pose.pose.orientation.w), rospy.Time.now(),
        #                    "calibration_target_pose", pose.header.frame_id)
        # rospy.sleep(.2)
        self.go_to_pose_goal(robot_name, ps_approach,speed=speed,end_effector_link=end_effector_link, move_lin = move_lin)
      if go_home:
        self.go_to_named_pose(home_pose, robot_name)
    
    if go_home:
      rospy.loginfo("Moving all robots home again.")
      self.go_to_named_pose("home", "a_bot")
      self.go_to_named_pose("home", "b_bot")
      self.go_to_named_pose("home", "c_bot")
    return
  
  def check_robot_calibration(self, position=""):
    calib_pose = geometry_msgs.msg.PoseStamped()
    if position == "":
      calib_pose.header.frame_id = "workspace_center"
      calib_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, 0))
      calib_pose.pose.position.x = -0.2
      calib_pose.pose.position.z = 0.07
    if position == "assembly_corner_4":
      calib_pose.header.frame_id = "assembled_assy_part_01_corner_4"
      calib_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, 0, 0))
      calib_pose.pose.position.x = -0.01
    if position == "b_c_corner":
      calib_pose.header.frame_id = "workspace_center"
      calib_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, 0))
      calib_pose.pose.position.x = -0.25
      calib_pose.pose.position.y = 0.3
      calib_pose.pose.position.z = 0.07
    if position == "a_b_corner":
      calib_pose.header.frame_id = "workspace_center"
      calib_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, 0))
      calib_pose.pose.position.x = -0.25
      calib_pose.pose.position.y = -0.3
      calib_pose.pose.position.z = 0.07
    

    rospy.loginfo("============ Testing robot calibration. ============")
    rospy.loginfo("Each robot will move to this position in front of c_bot:")
    rospy.loginfo(calib_pose)

    self.go_to_named_pose("back", "a_bot")
    self.go_to_named_pose("back", "b_bot")

    rospy.loginfo("============ Press `Enter` to move c_bot to calibration position, enter 0 to skip.")
    if raw_input() != "0":
      self.send_gripper_command("c_bot", "close")
      self.go_to_pose_goal("c_bot", calib_pose, speed=1.0)

    rospy.loginfo("============ Press `Enter` to move b_bot to calibration position, enter 0 to skip.")
    if raw_input() != "0":
      self.go_to_named_pose("back", "c_bot")
      self.send_gripper_command("b_bot", "close")
      self.go_to_pose_goal("b_bot", calib_pose, speed=1.0)

    rospy.loginfo("============ Press `Enter` to move a_bot to calibration position, enter 0 to skip.")
    if raw_input() != "0":
      self.go_to_named_pose("back", "b_bot")
      self.send_gripper_command("precision_gripper_inner", "close")
      self.go_to_pose_goal("a_bot", calib_pose, speed=1.0)

    rospy.loginfo("============ Press `Enter` to move robots back home.")
    raw_input()
    self.go_to_named_pose("home", "a_bot")
    self.go_to_named_pose("home", "b_bot")
    self.go_to_named_pose("home", "c_bot")
    return

  def check_c_bot_calibration(self):
    rospy.loginfo("============ Going to 5 mm above workspace center points with c_bot. ============")
    self.go_to_named_pose("back", "b_bot")
    self.go_to_named_pose("back", "a_bot")
    self.go_to_named_pose("home", "c_bot")
    poses = []

    pose0 = geometry_msgs.msg.PoseStamped()
    pose0.header.frame_id = "workspace_center"
    pose0.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, 0))
    pose0.pose.position.x = -.3
    pose0.pose.position.z = .005
    
    for i in range(4):
      poses.append(copy.deepcopy(pose0))

    poses[1].pose.position.x = 0.0
    
    poses[2].pose.position.y = -.3
    poses[2].pose.position.x = -.3
    
    poses[3].pose.position.y = .3

    self.cycle_through_calibration_poses(poses, "c_bot", speed=0.3, go_home=True)
    return

  def check_b_bot_calibration(self):
    rospy.loginfo("============ Going to 5 mm above workspace center points with b_bot. ============")
    self.go_to_named_pose("back", "c_bot")
    self.go_to_named_pose("back", "a_bot")
    self.go_to_named_pose("home", "b_bot")
    poses = []

    pose0 = geometry_msgs.msg.PoseStamped()
    pose0.header.frame_id = "workspace_center"
    pose0.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, 0))
    pose0.pose.position.z = .005
    
    for i in range(7):
      poses.append(copy.deepcopy(pose0))

    poses[1].pose.position.y = .3
    poses[2].pose.position.y = -.3
    
    poses[3].pose.position.y = 0.0
    poses[3].pose.position.x = -.3
    poses[4].pose.position.x = .3

    poses[5].pose.position.y = -.3
    poses[5].pose.position.x = -.3
    poses[6].pose.position.x = .3

    self.cycle_through_calibration_poses(poses, "b_bot", speed=0.3, go_home=True)
    return
  
  def align_c_b(self, part):
    rospy.loginfo("============ Testing calibration between c and b bot. ============")
    rospy.loginfo("c_bot will move to a bose, b_bot will go close to c_bot's gripper")

    self.go_to_named_pose("home", "a_bot")
    self.go_to_named_pose("home", "b_bot")
    self.go_to_named_pose("home", "c_bot")

    c_pose = geometry_msgs.msg.PoseStamped()
    if part == 1:
      c_pose.header.frame_id = "workspace_center"
      c_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, pi/2))
      c_pose.pose.position.x = -0.2
      c_pose.pose.position.z = 0.15
    elif part == 2:
      c_pose.header.frame_id = "screw_tool_m4_helper_link"
      c_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, pi))
      c_pose.pose.position.x = 0.0
      c_pose.pose.position.z = 0.08

      self.groups["b_bot"].set_joint_value_target([-30.0 * pi/180.0, -48 * pi/180.0, 96 * pi/180.0, 
                                    -50 * pi/180.0, -27 * pi/180.0, -180 * pi/180.0])
      self.groups["b_bot"].set_max_velocity_scaling_factor(.2)
      self.groups["b_bot"].go(wait=True)
      self.groups["b_bot"].stop()
      
    elif part == 3:
      c_pose.header.frame_id = "workspace_center"
      c_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(-pi/2, 0, 0))
      c_pose.pose.position.x = 0.0
      c_pose.pose.position.z = 0.6
      
    b_pose = geometry_msgs.msg.PoseStamped()
    b_pose.header.frame_id = "c_bot_robotiq_85_tip_link"
    b_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(pi, pi/2, 0))
    b_pose.pose.position.x = -0.02
    b_pose.pose.position.z = 0.02

    self.send_gripper_command("c_bot", "close")
    self.go_to_pose_goal("c_bot", c_pose, speed=1.0)
    rospy.sleep(1)
    self.send_gripper_command("b_bot", "close")
    self.move_lin("b_bot", b_pose, speed=0.05)

    rospy.loginfo("============ Press `Enter` to move robots back home.")
    if raw_input() != "0":
      self.go_to_named_pose("home", "c_bot", speed=0.1)
      self.go_to_named_pose("home", "b_bot", speed=0.1)
    return
  
  def taskboard_calibration(self):
    rospy.loginfo("============ Calibrating taskboard. ============")
    poses = []

    pose0 = geometry_msgs.msg.PoseStamped()
    pose0.header.frame_id = "taskboard_corner2"
    pose0.pose.orientation = self.a_bot_downward_orientation
    pose0.pose.position.z = .003

    for i in range(3):
      poses.append(copy.deepcopy(pose0))

    poses[1].header.frame_id = "taskboard_corner3"
    # poses[1].pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, pi/2))
    poses[2].header.frame_id = "taskboard_part10"
    
    self.cycle_through_calibration_poses(poses, "a_bot", speed=0.08)
    return

  def taskboard_calibration_extended(self, robot_name = "a_bot"):
    rospy.loginfo("============ Demonstrating the calibration of the taskboard. ============")
    rospy.loginfo("This moves " + robot_name + " to the top of the parts pre-mounted in the taskboard.")
    poses = []

    pose0 = geometry_msgs.msg.PoseStamped()
    pose0.pose.orientation = self.a_bot_downward_orientation
    pose0.pose.position.z = 0.002

    # On top of the metal sheet
    pose0 = copy.deepcopy(pose0)

    if robot_name == "a_bot":
      for i in range(4):
        poses.append(copy.deepcopy(pose0))

      poses[0].header.frame_id = "taskboard_part7_2"
      poses[0].pose.position.y = .0015
      poses[0].pose.position.z = .0253 + 0.0
      poses[1].header.frame_id = "taskboard_part8"
      poses[2].header.frame_id = "taskboard_part9"
      poses[3].header.frame_id = "taskboard_part14"

      self.cycle_through_calibration_poses(poses, robot_name, speed=0.3)
    elif robot_name == "c_bot":
      for i in range(4):
        poses.append(copy.deepcopy(pose0))

      poses[0].header.frame_id = "taskboard_part8"
      poses[1].header.frame_id = "taskboard_part9"
      poses[2].header.frame_id = "taskboard_part14"
      poses[3].header.frame_id = "taskboard_part5"

      self.cycle_through_calibration_poses(poses, robot_name, speed=0.1, move_lin = True)
    return

  def taskboard_calibration_mat(self, extended = False, robot_name = "a_bot"):
    rospy.loginfo("============ Calibrating placement mat for the taskboard task. ============")
    rospy.loginfo("a_bot gripper tip should be 3 mm above the surface.")
    self.go_to_named_pose("home", "a_bot", speed=0.1)
    self.go_to_named_pose("home", "b_bot", speed=0.1)
    poses = []

    pose0 = geometry_msgs.msg.PoseStamped()
    pose0.pose.orientation = self.a_bot_downward_orientation
    pose0.pose.position.z = .002
    
    if not extended: 
      for i in range(3):
        poses.append(copy.deepcopy(pose0))
      poses[0].header.frame_id = "mat_part15"
      poses[1].header.frame_id = "mat_part7_1"
      poses[2].header.frame_id = "mat_part3"
      self.cycle_through_calibration_poses(poses, robot_name, speed=0.3, move_lin = True)
    else:
      pose0.pose.position.z = .005
      for i in range(16):
        poses.append(copy.deepcopy(pose0))
      poses[0].header.frame_id = "mat_part1"
      poses[1].header.frame_id = "mat_part2"
      poses[2].header.frame_id = "mat_part3"
      poses[3].header.frame_id = "mat_part4"
      poses[4].header.frame_id = "mat_part5"
      poses[5].header.frame_id = "mat_part6"
      poses[6].header.frame_id = "mat_part7_1"
      poses[7].header.frame_id = "mat_part7_2"
      poses[8].header.frame_id = "mat_part8"
      poses[9].header.frame_id = "mat_part9"
      poses[10].header.frame_id = "mat_part10"
      poses[11].header.frame_id = "mat_part11"
      poses[12].header.frame_id = "mat_part12"
      poses[13].header.frame_id = "mat_part13"
      poses[14].header.frame_id = "mat_part14"
      poses[15].header.frame_id = "mat_part15"
      # Go to above center of mat quickly
      # above_mat = copy.deepcopy(pose0)
      # above_mat.header.frame_id = "mat"
      # above_
      # self.go_to_pose_goal(robot_name, ,speed=.1, move_lin = True)
      self.cycle_through_calibration_poses(poses, robot_name, speed=0.05, go_home=False, move_lin=True)
    return 

  def gripper_frame_calibration(self):
    rospy.loginfo("============ Calibrating the a_bot gripper tip frame for a_bot. ============")
    rospy.loginfo("Each approach of the target position has its orientation turned by 90 degrees.")
    poses = []

    pose0 = geometry_msgs.msg.PoseStamped()
    pose0.header.frame_id = "taskboard_part14"  # Good for demonstration, but not for calculation
    # pose0.header.frame_id = "mat_part10"      # Good for touching down and noting the position
    pose0.pose.orientation = self.a_bot_downward_orientation
    pose0.pose.position.x = -.002
    pose0.pose.position.y = -.002
    pose0.pose.position.z = .002

    q0 = tf_conversions.transformations.quaternion_from_euler(0, pi/2, pi)
    q_turn_90 = tf_conversions.transformations.quaternion_from_euler(pi/2, 0, 0)
    q1 = tf_conversions.transformations.quaternion_multiply(q0, q_turn_90)
    q2 = tf_conversions.transformations.quaternion_multiply(q1, q_turn_90)
    q3 = tf_conversions.transformations.quaternion_multiply(q2, q_turn_90)

    pose1 = copy.deepcopy(pose0)
    pose1.pose.orientation = geometry_msgs.msg.Quaternion(*q1)
    pose2 = copy.deepcopy(pose0)
    pose2.pose.orientation = geometry_msgs.msg.Quaternion(*q2)
    pose3 = copy.deepcopy(pose0)
    pose3.pose.orientation = geometry_msgs.msg.Quaternion(*q3)
    
    
    poses = [pose0, pose1, pose2, pose3]

    self.cycle_through_calibration_poses(poses, "a_bot", speed=0.3)
    return 

  def assembly_calibration_base_plate(self, robot_name="c_bot"):
    rospy.loginfo("============ Calibrating base plate for the assembly task. ============")
    rospy.loginfo(robot_name + "gripper tip should be 5 mm above each corner of the plate.")
    poses = []

    pose0 = geometry_msgs.msg.PoseStamped()
    pose0.pose.orientation.w = 1.0
    pose0.pose.position.x = -.01

    for i in range(4):
      poses.append(copy.deepcopy(pose0))
    poses[0].header.frame_id = "assembled_assy_part_03_bottom_screw_hole_aligner_2"
    poses[1].header.frame_id = "assembled_assy_part_03_bottom_screw_hole_aligner_1"
    poses[2].header.frame_id = "assembled_assy_part_01_corner_3"
    poses[3].header.frame_id = "assembled_assy_part_01_corner_4"

    self.cycle_through_calibration_poses(poses, robot_name, speed=0.3)
    return 

  def assembly_calibration_assembled_parts(self):
    rospy.loginfo("============ Calibrating full assembled parts for the assembly task. ============")
    rospy.loginfo("b_bot gripper tip should go close to some important spots.")
    poses = []

    pose0 = geometry_msgs.msg.PoseStamped()
    pose0.pose.orientation.w = 1.0
    pose0.pose.position.x = -.02

    for i in range(5):
      poses.append(copy.deepcopy(pose0))

    poses[1].header.frame_id = "assembled_assy_part_03"   # Top of plate 2
    poses[1].pose.position.x = .058
    poses[1].pose.position.y = -.0025
    poses[1].pose.position.z = .095 + .01
    poses[1].pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, 0) )

    poses[2].header.frame_id = "assembled_assy_part_08_front_tip"  # Front of rotary shaft
    poses[2].pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, 0, -pi) )
    poses[2].pose.position.x = .03

    poses[3].header.frame_id = "assembled_assy_part_14_screw_head"
    poses[3].pose.position.x = -.03

    poses[4].header.frame_id = "assembled_assy_part_04_tip"
    poses[4].pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(-pi, 0, -pi) )
    poses[4].pose.position.x = .03

    self.cycle_through_calibration_poses(poses, "b_bot", speed=0.3, go_home=True)
    return 
      
  def touch_the_table(self):
    rospy.loginfo("============ Calibrating robot heights. ============")
    self.go_to_named_pose("home", "a_bot")
    self.go_to_named_pose("home", "b_bot")
    self.go_to_named_pose("home", "c_bot")
    poses = []

    pose_a = geometry_msgs.msg.PoseStamped()
    pose_a.header.frame_id = "workspace_center"
    pose_a.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, 0))
    pose_a.pose.position.x = .0
    pose_a.pose.position.y = -.3
    pose_a.pose.position.z = .03

    pose_b = copy.deepcopy(pose_a)
    pose_b.pose.position.x = .0
    pose_b.pose.position.y = .3
    pose_b.pose.position.z = .03
    pose_c = copy.deepcopy(pose_a)
    pose_c.pose.position.x = -.3
    pose_c.pose.position.y = .2
    pose_c.pose.position.z = .03
    
    rospy.loginfo("============ Going to 2 cm above the table. ============")
    self.go_to_pose_goal("c_bot", pose_c, speed=1.0)
    self.go_to_pose_goal("b_bot", pose_b, speed=1.0)

    rospy.loginfo("============ Press enter to go to .1 cm above the table. ============")
    i = raw_input()
    if not rospy.is_shutdown():
      pose_b.pose.position.z = .001
      pose_c.pose.position.z = .001
      self.go_to_pose_goal("c_bot", pose_c, speed=0.01)
      self.go_to_pose_goal("b_bot", pose_b, speed=0.01)

    rospy.loginfo("============ Press enter to go home. ============")
    raw_input()
    self.go_to_named_pose("home", "a_bot")
    self.go_to_named_pose("home", "b_bot")
    self.go_to_named_pose("home", "c_bot")
    return

  def parts_tray_tests(self):
    rospy.loginfo("============ Going to parts tray positions with c_bot. ============")
    self.go_to_named_pose("home", "c_bot")
    poses = []
    pose0 = geometry_msgs.msg.PoseStamped()
    pose0.header.frame_id = "initial_assy_part_01"
    pose0.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, 0))
    pose0.pose.position.z = .03

    for i in range(6):
      poses.append(copy.deepcopy(pose0))

    poses[0].pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, 0, 0))
    poses[0].pose.position.z = .0
    poses[1].header.frame_id = "tray_2_screw_m3_5"
    poses[1].pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, 0, 0))
    poses[1].pose.position.z = .0
    poses[2].header.frame_id = "tray_2_partition_4"
    poses[3].header.frame_id = "tray_1_partition_1"
    poses[4].header.frame_id = "tray_1_partition_2"
    poses[5].header.frame_id = "tray_1_partition_5"

    self.cycle_through_calibration_poses(poses, "c_bot", speed=0.3, go_home=True)
    return
  
  def kitting_tray_test(self, robot_name = "a_bot", end_effector_link = ""):
    rospy.loginfo("============ Moving " + robot_name + "around the tray " + ", eef_link is " + end_effector_link + " ============")
    if robot_name=="b_bot":
      self.go_to_named_pose("back", "c_bot")
    elif robot_name=="a_bot":
      self.go_to_named_pose("back", "c_bot")

    self.go_to_named_pose("home", robot_name)

    poses = []
    setname = ""
    if task=="kitting":
      setname = "set_1_"

    pose0 = geometry_msgs.msg.PoseStamped()
    pose0.header.frame_id = setname + "tray_2_screw_m4_1"
    if robot_name=="b_bot":
      pose0.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(-pi, 0, 0))
      pose0.pose.position.x = -.01
    elif robot_name=="c_bot":
      pose0.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(-pi/2, 0, 0))
      pose0.pose.position.x = -.01
    elif robot_name=="a_bot":
      pose0.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(-pi, 0, 0))
      pose0.pose.position.x = -.01
    for i in range(8):
      poses.append(copy.deepcopy(pose0))

    poses[0].pose.position.x = -.05
    poses[2].header.frame_id = setname + "tray_2_screw_m4_4"
    poses[3].header.frame_id = setname + "tray_2_screw_m4_7"
    poses[4].header.frame_id = setname + "tray_2_screw_m3_1"
    poses[5].header.frame_id = setname + "tray_2_screw_m3_4"
    poses[6].header.frame_id = setname + "tray_2_screw_m4_3"
    poses[7].header.frame_id = setname + "tray_2_screw_m3_6"

    self.cycle_through_calibration_poses(poses, robot_name, speed=0.3, go_home=False, end_effector_link=end_effector_link, move_lin=True)
    self.go_to_named_pose("home", robot_name)
    return

  def kitting_trays_screw_test(self, robot_name = "c_bot", end_effector_link="c_bot_screw_tool_m4_tip_link"):
    rospy.loginfo("============ Moving " + robot_name + " " + end_effector_link + " to the screws in all the 3 trays (for the kitting task) ============")
    if robot_name=="c_bot":
      self.go_to_named_pose("back", "b_bot")
      self.go_to_named_pose("back", "a_bot")

    set_names = ["set_1_", "set_2_", "set_3_"]
    poses = []
    for setname in set_names:
      poses_set = []

      pose0 = geometry_msgs.msg.PoseStamped()
      pose0.header.frame_id = setname + "tray_2_screw_m4_1"
      
      if setname == "set_1_":
        pose0.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(pi/2, 0, 0))
      elif setname == "set_2_":
        pose0.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(pi/4, 0, 0))
      elif setname == "set_3_":
        pose0.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, 0, 0))
      
      pose0.pose.position.x = -.01
      
      for i in range(8):
        poses_set.append(copy.deepcopy(pose0))

      poses_set[0].pose.position.x = -.05
      poses_set[2].header.frame_id = setname + "tray_2_screw_m4_4"
      poses_set[3].header.frame_id = setname + "tray_2_screw_m4_7"
      poses_set[4].header.frame_id = setname + "tray_2_screw_m3_1"
      poses_set[5].header.frame_id = setname + "tray_2_screw_m3_4"
      poses_set[6].header.frame_id = setname + "tray_2_screw_m4_3"
      poses_set[7].header.frame_id = setname + "tray_2_screw_m3_6"
      poses.append(poses_set)

    self.go_to_named_pose("feeder_pick_ready", robot_name)
    self.cycle_through_calibration_poses(poses[0], robot_name, speed=0.3, go_home=False, move_lin=True, end_effector_link=end_effector_link)
    self.go_to_named_pose("feeder_pick_ready", robot_name)
    self.cycle_through_calibration_poses(poses[1], robot_name, speed=0.3, go_home=False, move_lin=True, end_effector_link=end_effector_link)
    self.go_to_named_pose("screw_place_ready_near_b_bot", robot_name)
    self.cycle_through_calibration_poses(poses[2], robot_name, speed=0.3, go_home=False, move_lin=True, end_effector_link=end_effector_link)
    return

  def place_screw_test(self, set_name = "set_1_", screw_size = 4, screw_number = 1):
    rospy.loginfo("============ Placing a screw in a tray using c_bot ============")
    rospy.loginfo("============ Screw tool m4 and a screw have to be carried by the robot! ============")
    self.go_to_named_pose("back", "a_bot")
    self.go_to_named_pose("back", "b_bot")

    ps = geometry_msgs.msg.PoseStamped()
    ps.header.frame_id = set_name + "tray_2_screw_m" + str(screw_size) + "_" + str(screw_number)
    if set_name == "set_1_":
      self.go_to_named_pose("feeder_pick_ready", "c_bot")
      ps.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(pi/2, 0, 0))
    elif set_name == "set_2_":
      self.go_to_named_pose("feeder_pick_ready", "c_bot")
      ps.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(pi/4, 0, 0))
    elif set_name == "set_3_":
      self.go_to_named_pose("screw_place_ready_near_b_bot", "c_bot")
      ps.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, 0, 0))
    
    self.do_place_action("c_bot", ps, tool_name="screw_tool", screw_size = 4)
    return
  
  def assembly_calibration_initial_plates(self):
    rospy.loginfo("============ Going to above plate 2 and 3 with c_bot. ============")
    self.go_to_named_pose("home", "c_bot")
    poses = []

    pose0 = geometry_msgs.msg.PoseStamped()
    pose0.header.frame_id = "initial_assy_part_03"
    pose0.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, 0))
    pose0.pose.position.x = .116
    pose0.pose.position.z = .095
    
    for i in range(2):
      poses.append(copy.deepcopy(pose0))

    poses[1].header.frame_id = "initial_assy_part_02"
    poses[1].pose.position.x = .07
    poses[1].pose.position.z = .065

    self.cycle_through_calibration_poses(poses, "c_bot", speed=0.3, go_home=True)
    return

  def screw_tool_tests(self):
    rospy.loginfo("============ Calibrating screw_tool M4 with b_bot. ============")
    self.go_to_named_pose("screw_ready", "b_bot")
    poses = []

    ps = geometry_msgs.msg.PoseStamped()
    ps.header.frame_id = "workspace_center"
    ps.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi, 0))
    ps.pose.position.x = .0
    ps.pose.position.y = .0
    ps.pose.position.z = .05

    rospy.loginfo("============ Press enter to hold tool vertically. ============")
    i = raw_input()

    ps.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, -pi/2))
    ps.pose.position.x = -.01
    ps.pose.position.y = .0
    ps.pose.position.z = .05
    self.publish_marker(ps, "pose")
    self.groups["b_bot"].set_pose_target(ps, end_effector_link="b_bot_screw_tool_m4_tip_link")
    self.groups["b_bot"].set_max_velocity_scaling_factor(.05)
    self.groups["b_bot"].go()
    self.groups["b_bot"].stop()
    self.groups["b_bot"].clear_pose_targets()

    rospy.loginfo("============ Press enter to go home. ============")
    raw_input()
    self.go_to_named_pose("screw_ready", "b_bot")
    return

  def screw_holder_tests(self, robot_name="b_bot"):
    rospy.loginfo("============ Going to screw tool holder with " + robot_name + ". ============")

    if robot_name == "b_bot":
      self.groups["b_bot"].set_joint_value_target([-30.0 * pi/180.0, -48 * pi/180.0, 96 * pi/180.0, 
                                      -50 * pi/180.0, -27 * pi/180.0, -180 * pi/180.0])
      self.groups["b_bot"].set_max_velocity_scaling_factor(.2)
      self.groups["b_bot"].go(wait=True)
      self.groups["b_bot"].stop()

    poses = []
    pose0 = geometry_msgs.msg.PoseStamped()
    pose0.header.frame_id = "screw_tool_m4_helper_link"
    if robot_name == "b_bot":
      pose0.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, 0, 0))
      pose0.pose.position.x -= .03
      pose0.pose.position.z = .017
    elif robot_name == "c_bot":
      pose0.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, pi))
      pose0.pose.position.z = .02
    
    for i in range(3):
      poses.append(copy.deepcopy(pose0))

    poses[1].header.frame_id = "screw_tool_m3_helper_link"
    poses[2].header.frame_id = "screw_tool_m6_helper_link"

    self.cycle_through_calibration_poses(poses, robot_name, speed=0.3, go_home=False)
    return

  def screw_tool_test_assembly(self, robot_name = "b_bot", tool_name="_screw_tool_m4_tip_link"):
    rospy.loginfo("============ Moving the screw tool m4 to the four corners of the base plate ============")
    rospy.loginfo("============ The screw tool m4 has to be carried by the robot! ============")
    if robot_name=="b_bot":
      self.go_to_named_pose("back", "c_bot")
    elif robot_name=="c_bot":
      self.go_to_named_pose("back", "b_bot")

    self.go_to_named_pose("screw_ready", robot_name)
    poses = []

    pose0 = geometry_msgs.msg.PoseStamped()
    pose0.header.frame_id = "assembled_assy_part_01_corner_2"
    if robot_name=="b_bot":
      pose0.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(pi/4, 0, 0))
      pose0.pose.position.x -= .02
    elif robot_name=="c_bot":
      pose0.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(pi/2, 0, 0))
      pose0.pose.position.x -= .02
    
    for i in range(4):
      poses.append(copy.deepcopy(pose0))

    poses[1].header.frame_id = "assembled_assy_part_01_corner_3"
    poses[2].header.frame_id = "assembled_assy_part_01_corner_4"
    poses[3].header.frame_id = "assembled_assy_part_01_corner_1"
    end_effector_link=robot_name+ tool_name
    self.cycle_through_calibration_poses(poses, robot_name, speed=0.3, go_home=False, move_lin=True, end_effector_link=end_effector_link)
    return

  def tray_screw_calibration(self, robot_name = "b_bot", end_effector_link="", task="assembly", set_number=1):
    rospy.loginfo("============ Moving " + robot_name + " " + end_effector_link + " to the screws in the tray ============")
    if robot_name=="b_bot":
      self.go_to_named_pose("back", "c_bot")
    elif robot_name=="c_bot":
      self.go_to_named_pose("back", "b_bot")

    if end_effector_link=="":
      self.go_to_named_pose("home", robot_name)
    elif "screw" in end_effector_link:
      self.go_to_named_pose("screw_ready", robot_name)
    elif "suction" in end_effector_link:
      self.go_to_named_pose("screw_ready", robot_name)

    poses = []

    pose0 = geometry_msgs.msg.PoseStamped()
    pose0.header.frame_id = "set_" + str(set_number) + "_tray_2_screw_m4_1"
    pose0.pose.position.x = -.01
    if robot_name=="b_bot" or robot_name=="a_bot":
      pose0.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(-pi/2, 0, 0))
      if task=="kitting":
        pose0.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(-pi/2, 0, 0))
    elif robot_name=="c_bot":
      if set_number == 1:
        pose0.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(pi/2, 0, 0))
      elif set_number == 2:
        pose0.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(pi/4, 0, 0))
      elif set_number == 3:
        pose0.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, 0, 0))
    
    for i in range(8):
      poses.append(copy.deepcopy(pose0))

    poses[0].pose.position.x = -.05
    poses[2].header.frame_id = "set_" + str(set_number) + "_tray_2_screw_m4_4"
    poses[3].header.frame_id = "set_" + str(set_number) + "_tray_2_screw_m4_7"
    poses[4].header.frame_id = "set_" + str(set_number) + "_tray_2_screw_m3_1"
    poses[5].header.frame_id = "set_" + str(set_number) + "_tray_2_screw_m3_4"
    poses[6].header.frame_id = "set_" + str(set_number) + "_tray_2_screw_m4_3"
    poses[7].header.frame_id = "set_" + str(set_number) + "_tray_2_screw_m3_6"

    self.cycle_through_calibration_poses(poses, robot_name, speed=0.3, go_home=False, move_lin=True, end_effector_link=end_effector_link)
    return

  def screw_tool_test_tray_with_fingertip(self, robot_name = "b_bot"):
    rospy.loginfo("============ Moving robot finger tip to the screw on the tray============")
    if robot_name=="b_bot":
      self.go_to_named_pose("back", "c_bot")
    elif robot_name=="a_bot":
      self.go_to_named_pose("back", "c_bot")

    self.go_to_named_pose("home", robot_name)

    poses = []

    pose0 = geometry_msgs.msg.PoseStamped()
    pose0.header.frame_id = "tray_2_screw_m4_1"
    if robot_name=="b_bot":
      pose0.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(-pi, 0, 0))
      pose0.pose.position.x = -.01
    elif robot_name=="c_bot":
      pose0.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(-pi/2, 0, 0))
      pose0.pose.position.x = -.01
    elif robot_name=="a_bot":
      pose0.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(-pi, 0, 0))
      pose0.pose.position.x = -.01
    for i in range(8):
      poses.append(copy.deepcopy(pose0))

    poses[0].pose.position.x = -.15
    poses[2].header.frame_id = "tray_2_screw_m4_4"
    poses[3].header.frame_id = "tray_2_screw_m4_7"
    poses[4].header.frame_id = "tray_2_screw_m3_1"
    poses[5].header.frame_id = "tray_2_screw_m3_4"
    poses[6].header.frame_id = "tray_2_screw_m4_3"
    poses[7].header.frame_id = "tray_2_screw_m3_6"

    self.cycle_through_calibration_poses(poses, robot_name, speed=0.3, go_home=False, move_lin=True)
    return

  
  def screw_pickup_test(self, robot_name = "b_bot"):
    rospy.loginfo("============ Picking up an m4 screw with the tool ============")
    rospy.loginfo("============ The screw tool m4 has to be carried by the robot! ============")
    if robot_name=="b_bot":
      self.go_to_named_pose("back", "c_bot")
    elif robot_name=="c_bot":
      self.go_to_named_pose("back", "b_bot")

    self.go_to_named_pose("screw_ready", robot_name)
    if robot_name=="b_bot":
      self.go_to_named_pose("screw_pick_ready", robot_name)

    pose0 = geometry_msgs.msg.PoseStamped()
    pose0.header.frame_id = "tray_2_screw_m4_1"
    if robot_name=="b_bot":
      pose0.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(-pi/2, 0, 0))
      pose0.pose.position.x = -.01
    elif robot_name=="c_bot":
      pose0.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(-pi/2, 0, 0))
      pose0.pose.position.x = -.01
    
    self.do_pick_action(robot_name, pose0, screw_size = 4, z_axis_rotation = 0.0, use_complex_planning = True, tool_name = "screw_tool")
    return
  
  def screw_action_test(self, robot_name = "b_bot"):
    rospy.loginfo("============ Screwing in one of the plate screws with the tool using the action ============")
    rospy.loginfo("============ The screw tool m4 and a screw have to be carried by the robot! ============")
    if robot_name=="b_bot":
      self.go_to_named_pose("back", "c_bot")
    elif robot_name=="c_bot":
      self.go_to_named_pose("back", "b_bot")

    self.go_to_named_pose("screw_ready", robot_name)

    if robot_name=="b_bot":
      self.go_to_named_pose("screw_plate_ready", robot_name)
    elif robot_name=="c_bot":
      self.go_to_named_pose("screw_ready_high", robot_name)

    pose0 = geometry_msgs.msg.PoseStamped()
    if robot_name=="b_bot":
      pose0.header.frame_id = "assembled_assy_part_03_bottom_screw_hole_aligner_1"
      pose0.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(-pi/4, 0, 0))
      pose0.pose.position.x = -.01
    elif robot_name=="c_bot":
      pose0.header.frame_id = "assembled_assy_part_11_screw_head_2"
      pose0.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(pi, 0, 0))
      pose0.pose.position.x = -.08
      self.go_to_pose_goal(robot_name, pose0,speed=.3,end_effector_link=robot_name + "_screw_tool_m4_tip_link", move_lin = True)
      pose0.pose.position.x = -.01
    
    self.do_screw_action(robot_name, pose0, screw_size = 4, screw_height = .02)
    if robot_name=="b_bot":
      self.go_to_named_pose("screw_plate_ready", robot_name)
    elif robot_name=="c_bot":
      pose0.pose.position.x = -.08
      self.go_to_pose_goal(robot_name, pose0,speed=.3,end_effector_link=robot_name + "_screw_tool_m4_tip_link", move_lin = True)
    return

  def screw_feeder_calibration(self, robot_name = "c_bot"):
    rospy.loginfo("============ Moving the screw tool m4 to the screw feeder ============")
    rospy.loginfo("============ The screw tool m4 has to be carried by the robot! ============")
    if robot_name=="c_bot":
      self.go_to_named_pose("back", "b_bot")
      self.go_to_named_pose("back", "a_bot")

    self.go_to_named_pose("feeder_pick_ready", robot_name)
    
    # Turn to the right
    self.groups["c_bot"].set_joint_value_target([0, -2.0980, 1.3992, -1.6153, -1.5712, -3.1401])
    self.groups["c_bot"].set_max_velocity_scaling_factor(1.0)
    self.groups["c_bot"].go(wait=True)
    self.groups["c_bot"].stop()

    poses = []
    pose0 = geometry_msgs.msg.PoseStamped()
    self.toggle_collisions(collisions_on=False)
    pose0.header.frame_id = "m3_feeder_outlet_link"
    if robot_name=="c_bot":
      pose0.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, 0, 0))
      pose0.pose.position.x = -.03
    
    for i in range(6):
      poses.append(copy.deepcopy(pose0))

    poses[1].pose.position.x = 0

    poses[3].header.frame_id = "m4_feeder_outlet_link"
    poses[4].header.frame_id = "m4_feeder_outlet_link"
    poses[4].pose.position.x = 0
    poses[5].header.frame_id = "m4_feeder_outlet_link"
    
    self.cycle_through_calibration_poses(poses, robot_name, speed=0.3, go_home=False, move_lin=True, end_effector_link=robot_name + "_screw_tool_m4_tip_link")
    self.toggle_collisions(collisions_on=True)
    return
  
  def screw_feeder_pick_test(self, robot_name = "c_bot", screw_size = 4):
    rospy.loginfo("============ Picking a screw from a feeder ============")
    rospy.loginfo("============ The screw tool has to be carried by c_bot! ============")
    if robot_name=="c_bot":
      self.go_to_named_pose("back", "b_bot")
      self.go_to_named_pose("back", "a_bot")

    self.go_to_named_pose("feeder_pick_ready", robot_name)
    
    # Turn to the right
    self.groups["c_bot"].set_joint_value_target([0, -2.0980, 1.3992, -1.6153, -1.5712, -3.1401])
    self.groups["c_bot"].set_max_velocity_scaling_factor(1.0)
    self.groups["c_bot"].go(wait=True)
    self.groups["c_bot"].stop()

    ps = geometry_msgs.msg.PoseStamped()
    ps.header.frame_id = "m" + str(screw_size) + "_feeder_outlet_link"
    if robot_name=="c_bot":
      ps.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, 0, 0))
    
    self.do_pick_action("c_bot", ps, screw_size=screw_size, tool_name="screw_tool")
    return

  def tray_partition_calibration(self, robot_name="b_bot", end_effector_link="", task="assembly", set_number=1, tray_number=1):
    rospy.loginfo("============ Calibrating trays. ============")
    rospy.loginfo(robot_name + " end effector should be 3 cm above tray partition.")
    if robot_name=="b_bot":
      self.go_to_named_pose("back", "c_bot")
    elif robot_name=="c_bot":
      self.go_to_named_pose("back", "b_bot")
    
    if end_effector_link=="":
      self.go_to_named_pose("home", robot_name)
    elif "screw" in end_effector_link:
      self.go_to_named_pose("screw_ready", robot_name)
    elif "suction" in end_effector_link:
      self.go_to_named_pose("screw_ready", robot_name)

    poses = []

    pose0 = geometry_msgs.msg.PoseStamped()

    if tray_number == 2:
      pose0.header.frame_id = "set_" + str(set_number) + "_tray_2_partition_1"
      pose0.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, 0))
      if end_effector_link and robot_name == "b_bot" and task=="kitting":
        pose0.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, -pi/2))

      pose0.pose.position.x = 0
      pose0.pose.position.z = 0.03

      for i in range(8):
        poses.append(copy.deepcopy(pose0))

      poses[1].header.frame_id = "set_" + str(set_number) + "_tray_2_partition_2"
      poses[2].header.frame_id = "set_" + str(set_number) + "_tray_2_partition_3"
      poses[3].header.frame_id = "set_" + str(set_number) + "_tray_2_partition_4"
      poses[4].header.frame_id = "set_" + str(set_number) + "_tray_2_partition_5"
      poses[5].header.frame_id = "set_" + str(set_number) + "_tray_2_partition_6"
      poses[6].header.frame_id = "set_" + str(set_number) + "_tray_2_partition_7"
      poses[7].header.frame_id = "set_" + str(set_number) + "_tray_2_partition_8"
    
    elif tray_number == 1:
      pose0.header.frame_id = "set_" + str(set_number) + "_tray_1_partition_1"
      pose0.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, pi))
      if end_effector_link and robot_name == "b_bot" and task=="kitting":
        pose0.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, pi))
      pose0.pose.position.x = 0
      pose0.pose.position.z = 0.05
      for i in range(5):
        poses.append(copy.deepcopy(pose0))

      poses[0].header.frame_id = "set_" + str(set_number) + "_tray_1_partition_1"
      poses[1].header.frame_id = "set_" + str(set_number) + "_tray_1_partition_2"
      poses[2].header.frame_id = "set_" + str(set_number) + "_tray_1_partition_3"
      poses[3].header.frame_id = "set_" + str(set_number) + "_tray_1_partition_4"
      poses[4].header.frame_id = "set_" + str(set_number) + "_tray_1_partition_5"

    self.cycle_through_calibration_poses(poses, robot_name, speed=0.1, end_effector_link=end_effector_link, move_lin=True, go_home=False)
    return 

  

  def bin_calibration(self, robot_name="b_bot", end_effector_link=""):
    rospy.loginfo("============ Calibrating bins. ============")
    rospy.loginfo(robot_name + " end effector should be 10 cm above center of bin.")
    if robot_name=="a_bot":
      self.go_to_named_pose("back", "b_bot")
    elif robot_name=="b_bot":
      self.go_to_named_pose("back", "c_bot")
    elif robot_name=="c_bot":
      self.go_to_named_pose("back", "b_bot")
    
    if end_effector_link=="":
      self.go_to_named_pose("home", robot_name)
    elif "screw" in end_effector_link:
      self.go_to_named_pose("screw_ready", robot_name)
    elif "suction" in end_effector_link:
      self.go_to_named_pose("screw_ready", robot_name)

    poses = []

    pose0 = geometry_msgs.msg.PoseStamped()
    # pose0.header.frame_id = "bin2_1"
    pose0.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, 0))
    if end_effector_link and robot_name == "b_bot":
      pose0.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, -pi/2))
    pose0.pose.position.z = 0.1

    for bin in self.bin_names:
      pose0.header.frame_id = bin
      world_pose = self.listener.transformPose("workspace_center", pose0)
      if robot_name == "a_bot":
        if world_pose.pose.position.y > .15:
          continue
      if robot_name == "b_bot":
        if world_pose.pose.position.y < -.15:
          continue
      poses.append(copy.deepcopy(pose0))

    self.cycle_through_calibration_poses(poses, robot_name, speed=0.1, end_effector_link=end_effector_link, move_lin=True, go_home=False)
    return 

  def bin_corner_calibration(self, robot_name="b_bot", end_effector_link=""):
    rospy.loginfo("============ Calibrating bin. ============")
    rospy.loginfo(robot_name + " end effector should be 3 cm above each corner of each bin.")
    if robot_name=="a_bot":
      self.go_to_named_pose("back", "b_bot")
    elif robot_name=="b_bot":
      self.go_to_named_pose("back", "c_bot")
    elif robot_name=="c_bot":
      self.go_to_named_pose("back", "b_bot")
    
    if end_effector_link=="":
      self.go_to_named_pose("home", robot_name)
    elif "screw" in end_effector_link:
      self.go_to_named_pose("screw_ready", robot_name)
    elif "suction" in end_effector_link:
      self.go_to_named_pose("screw_ready", robot_name)

    poses = []

    pose0 = geometry_msgs.msg.PoseStamped()
    pose0.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, 0))
    if end_effector_link and robot_name == "b_bot":
      pose0.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, -pi/2))
    pose0.pose.position.z = 0.03

    for bin in self.bin_names:
      pose0.header.frame_id = bin
      world_pose = self.listener.transformPose("workspace_center", pose0)
      if robot_name == "a_bot":
        if world_pose.pose.position.y > .1:
          continue
      if robot_name == "b_bot":
        if world_pose.pose.position.y < -.1:
          continue
      new_pose = copy.deepcopy(pose0)
      new_pose.header.frame_id = bin + "_top_back_left_corner"
      poses.append(new_pose)
      new_pose = copy.deepcopy(pose0)
      new_pose.header.frame_id = bin + "_top_back_right_corner"
      poses.append(new_pose)
      new_pose = copy.deepcopy(pose0)
      new_pose.header.frame_id = bin + "_top_front_right_corner"
      poses.append(new_pose)
      new_pose = copy.deepcopy(pose0)
      new_pose.header.frame_id = bin + "_top_front_left_corner"
      poses.append(new_pose)

    self.cycle_through_calibration_poses(poses, robot_name, speed=0.1, end_effector_link=end_effector_link, move_lin=True, go_home=False)
    return 

  def check_inner_pick_calibration(self):
    #Go to check pose
    self.go_to_named_pose("check_precision_gripper_success", "a_bot")
    self.send_gripper_command("precision_gripper_inner", "close")
    rospy.sleep(1.0)
    
    cv2.imwrite('/root/catkin_ws/src/o2as_bg_ratio/images/empty_close_gripper.png', self._img)
    return

  def image_callback(self, msg_in):
    self._img = self.bridge.imgmsg_to_cv2(msg_in, desired_encoding="passthrough")


if __name__ == '__main__':
  try:
    c = CalibrationClass()

    while not rospy.is_shutdown():
      rospy.loginfo("============ Calibration procedures ============ ")
      rospy.loginfo("Enter a number to check calibrations for the following things: ")
      rospy.loginfo("1: The robots (central position with nothing on the table)")
      rospy.loginfo("1000: Move with different jerk values")
      rospy.loginfo("100 (1001): Go home with all robots (using UR script joint move)")
      rospy.loginfo("101 (1011): Go back with all robots (using UR script joint move)")
      rospy.loginfo("111: The robots (Using the assembly base plate)")
      rospy.loginfo("112: The robots (Using the corner between b/c)")
      rospy.loginfo("113: The robots (Using the corner between a/b)")
      rospy.loginfo("12: Touch the table (all bots)")
      rospy.loginfo("122: Go to different spots on table with c_bot")
      rospy.loginfo("123: Go to different spots on table with b_bot")
      rospy.loginfo("13: Align c/b grippers part 1 (near board)")
      rospy.loginfo("14: Align c/b grippers part 2 (by holders)")
      rospy.loginfo("15: Align c/b grippers part 3 (high up)")
      rospy.loginfo("2: Taskboard (a_bot)")
      rospy.loginfo("21: Taskboard extended fun tour")
      rospy.loginfo("211: Taskboard extended with b_bot")
      rospy.loginfo("22: Placement mat (for the taskboard task)")
      rospy.loginfo("23: Placement mat extended (for the taskboard task)")
      rospy.loginfo("231: Placement mat extended with b_bot")
      rospy.loginfo("24: a_bot gripper frame (rotate around EEF axis on taskboard)")
      rospy.loginfo("3: ===== KITTING TASK (no action)")
      rospy.loginfo("311, 312, 313: Set 1 partitions with a_bot, b_bot, c_bot")
      rospy.loginfo("314, 315: Set 1 partitions with suction_tool (b_bot), screw_tool_m4 (c_bot)")
      rospy.loginfo("316 (3161, 3162): Screws in set 1 (2, 3) with screw_tool_m4 (c_bot)")
      rospy.loginfo("317 (3171, 3172): Screws in set 1 (2, 3) with screw_tool_m3 (c_bot)")
      rospy.loginfo("318, 3181, 3182: Screws in set 1 with b_bot_suction_tool, b_bot, a_bot")
      rospy.loginfo("3191, 3192: Tray 1 set 2, 3 partitions with b_bot")
      rospy.loginfo("321, 322, 323: Bins with a_bot, b_bot, suction_tool (b_bot)")
      rospy.loginfo("331, 332, 333: Bin corners with a_bot, b_bot, suction_tool (b_bot)")
      rospy.loginfo("341: Go to check pick pose and save image to file")
      rospy.loginfo("371: Go to screw feeder outlets with m4 tool for c_bot")
      rospy.loginfo("372, 373: Pick m4, m3 screw from feeder with c_bot (tool has to be equipped)")
      rospy.loginfo("4: Parts tray tests (assembly/kitting)")
      rospy.loginfo("5: ===== ASSEMBLY TASK (no action)")
      rospy.loginfo("51: Assembly base plate (c_bot)")
      rospy.loginfo("52: Assembly base plate (b_bot)")
      rospy.loginfo("53: Assembly base plate (a_bot)")
      rospy.loginfo("54: Assembly assembled parts")
      rospy.loginfo("55: Assembly initial part locations (the plates)")
      rospy.loginfo("6: ===== TOOLS|  Go to screw_ready with c and b (a goes to back)")
      rospy.loginfo("611, 612: Go to screw holder with b_bot, c_bot")
      rospy.loginfo("621, 622: Equip/unequip m4 tool (with b_bot)")
      rospy.loginfo("623, 624 (625, 626): Equip/unequip m4 (m3) tool (with c_bot)")
      rospy.loginfo("627, 628: Equip/unequip suction tool (with b_bot)")
      rospy.loginfo("63, 64: Go to assembly base plate with m4 screw tool (b_bot, c_bot)")
      rospy.loginfo("641: Go to assembly base plate with m6 nut tool (c_bot; m6 nut tool has to be equipped)")
      rospy.loginfo("65, 66: Go to tray positions with m4 tool for b_bot, c_bot (tool has to be equipped)")
      rospy.loginfo("661: Go to all 3 trays' positions with m4 tool for c_bot (tool has to be equipped)")
      rospy.loginfo("671: Pick up screw from tray with b_bot (tool has to be equipped)")
      rospy.loginfo("681: Place screw in tray 1 with c_bot (tool+screw required)")
      rospy.loginfo("6812: As above, with screw 2")
      rospy.loginfo("6813: As above, with screw 3")
      rospy.loginfo("682: Place screw in tray 2 with c_bot (tool+screw required)")
      rospy.loginfo("683: Place screw in tray 3 with c_bot (tool+screw required)")
      rospy.loginfo("691: Do screw action with b_bot on rightmost hole")
      rospy.loginfo("692: Do screw action with c_bot on pulley")
      rospy.loginfo("81: Go to tray partitions using b_bot")
      rospy.loginfo("82: Go to screws on the tray using b_bot")
      rospy.loginfo("83: Go to screws on the tray using a_bot")
      rospy.loginfo("x: Exit ")
      rospy.loginfo(" ")
      r = raw_input()
      if r == '1':
        c.check_robot_calibration()
      elif r == '1000':
        ps = geometry_msgs.msg.PoseStamped()
        ps.header.frame_id = "workspace_center"
        ps.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, pi))
        ps.pose.position.z = .2
        c.go_to_named_pose("home", "b_bot")
        c.move_lin("b_bot", ps, speed=.2, acceleration=.04)
        c.go_to_named_pose("home", "b_bot")
        c.move_lin("b_bot", ps, speed=.2, acceleration=.08)
        c.go_to_named_pose("home", "b_bot")
        c.move_lin("b_bot", ps, speed=.2, acceleration=.15)
        c.go_to_named_pose("home", "b_bot")
      elif r == '100':
        c.go_to_named_pose("home", "a_bot")
        c.go_to_named_pose("home", "b_bot")
        c.go_to_named_pose("home", "c_bot")
      elif r == '1001':
        c.go_to_named_pose("home", "a_bot", speed=3.0, acceleration=3.0, force_ur_script=True)
        c.go_to_named_pose("home", "b_bot", speed=3.0, acceleration=3.0, force_ur_script=True)
        c.go_to_named_pose("home", "c_bot", speed=3.0, acceleration=3.0, force_ur_script=True)
      elif r == '101':
        c.go_to_named_pose("back", "a_bot")
        c.go_to_named_pose("back", "b_bot")
        c.go_to_named_pose("back", "c_bot")
      elif r == '1011':
        c.go_to_named_pose("back", "a_bot", speed=3.0, acceleration=3.0, force_ur_script=True)
        c.go_to_named_pose("back", "b_bot", speed=3.0, acceleration=3.0, force_ur_script=True)
        c.go_to_named_pose("back", "c_bot", speed=3.0, acceleration=3.0, force_ur_script=True)
      elif r == '111':
        c.check_robot_calibration(position="assembly_corner_4")
      elif r == '112':
        c.check_robot_calibration(position="b_c_corner")
      elif r == '113':
        c.check_robot_calibration(position="a_b_corner")
      elif r == '12':
        c.touch_the_table()
      elif r == '122':
        c.check_c_bot_calibration()
      elif r == '123':
        c.check_b_bot_calibration()
      elif r == '13':
        c.align_c_b(part=1)
      elif r == '14':
        c.align_c_b(part=2)
      elif r == '15':
        c.align_c_b(part=3)
      elif r == '2':
        c.taskboard_calibration()
      elif r == '21':
        c.taskboard_calibration_extended()
      elif r == '211':
        c.taskboard_calibration_extended("b_bot")
      elif r == '22':
        c.taskboard_calibration_mat()
      elif r == '23':
        c.taskboard_calibration_mat(extended=True)
      elif r == '231':
        c.taskboard_calibration_mat(extended=True, robot_name="b_bot")
      elif r == '24':
        c.gripper_frame_calibration()
      elif r == '3':
        rospy.loginfo("NOT YET IMPLEMENTED")
      elif r == '311':
        c.tray_partition_calibration(robot_name="a_bot", set_number=1, tray_number=1)
        c.tray_partition_calibration(robot_name="a_bot", set_number=1, tray_number=2)
      elif r == '312':
        c.tray_partition_calibration(robot_name="b_bot", set_number=1, tray_number=1)
        c.tray_partition_calibration(robot_name="b_bot", set_number=1, tray_number=2)
      elif r == '313':
        c.tray_partition_calibration(robot_name="c_bot", set_number=1, tray_number=1)
        c.tray_partition_calibration(robot_name="c_bot", set_number=1, tray_number=2)
      elif r == '314':
        c.tray_partition_calibration(robot_name="b_bot", end_effector_link="b_bot_suction_tool_tip_link", task="kitting", set_number=1, tray_number=1)
        c.tray_partition_calibration(robot_name="b_bot", end_effector_link="b_bot_suction_tool_tip_link", task="kitting", set_number=1, tray_number=2)
      elif r == '315':
        c.tray_partition_calibration(robot_name="c_bot", end_effector_link="c_bot_screw_tool_m4_tip_link", task="kitting", set_number=1, tray_number=1)
        c.tray_partition_calibration(robot_name="c_bot", end_effector_link="c_bot_screw_tool_m4_tip_link", task="kitting", set_number=1, tray_number=2)
      elif r == '316':
        c.tray_screw_calibration(robot_name="c_bot", end_effector_link="c_bot_screw_tool_m4_tip_link", task="kitting", set_number=1)
      elif r == '3161':
        c.tray_screw_calibration(robot_name="c_bot", end_effector_link="c_bot_screw_tool_m4_tip_link", task="kitting", set_number=2)
      elif r == '3162':
        c.tray_screw_calibration(robot_name="c_bot", end_effector_link="c_bot_screw_tool_m4_tip_link", task="kitting", set_number=3)
      elif r == '317':
        c.tray_screw_calibration(robot_name="c_bot", end_effector_link="c_bot_screw_tool_m3_tip_link", task="kitting", set_number=1)
      elif r == '3171':
        c.tray_screw_calibration(robot_name="c_bot", end_effector_link="c_bot_screw_tool_m3_tip_link", task="kitting", set_number=2)
      elif r == '3172':
        c.tray_screw_calibration(robot_name="c_bot", end_effector_link="c_bot_screw_tool_m3_tip_link", task="kitting", set_number=3)
      elif r == '318':
        c.tray_screw_calibration(robot_name="b_bot", end_effector_link="b_bot_suction_tool_tip_link", task="kitting", set_number=1)
      elif r == '3181':
        c.tray_screw_calibration(robot_name="b_bot", task="kitting", set_number=1)
      elif r == '3182':
        c.tray_screw_calibration(robot_name="a_bot", task="kitting", set_number=1)
      elif r == '3191':
        c.tray_partition_calibration(robot_name="b_bot", set_number=2, tray_number=1)
      elif r == '3192':
        c.tray_partition_calibration(robot_name="b_bot", set_number=3, tray_number=1)
      elif r == '321':
        c.bin_calibration(robot_name="a_bot")
      elif r == '322':
        c.bin_calibration(robot_name="b_bot")
      elif r == '323':
        c.bin_calibration(robot_name="b_bot", end_effector_link="b_bot_suction_tool_tip_link")
      elif r == '331':
        c.bin_corner_calibration(robot_name="a_bot")
      elif r == '332':
        c.bin_corner_calibration(robot_name="b_bot")
      elif r == '333':
        c.bin_corner_calibration(robot_name="b_bot", end_effector_link="b_bot_suction_tool_tip_link")
      elif r == '341':
        c.check_inner_pick_calibration()
      elif r == '371':
        c.screw_feeder_calibration(robot_name="c_bot")
      elif r == '372':
        c.screw_feeder_pick_test(robot_name="c_bot", screw_size=4)
      elif r == '373':
        c.screw_feeder_pick_test(robot_name="c_bot", screw_size=3)
      elif r == '4':
        c.parts_tray_tests()  # Outdated, probably
      elif r == '51':
        c.assembly_calibration_base_plate()
      elif r == '52':
        c.assembly_calibration_base_plate("b_bot")
      elif r == '53':
        c.assembly_calibration_base_plate("a_bot")
      elif r == '54':
        c.assembly_calibration_assembled_parts()
      elif r == '55':
        c.assembly_calibration_initial_plates()
      elif r == '581':
        c.tray_partition_calibration(robot_name="b_bot")
      elif r == '582':
        c.screw_tool_test_tray_with_fingertip(robot_name="b_bot")
      elif r == '583':
        c.screw_tool_test_tray_with_fingertip(robot_name="a_bot")
      elif r == '6':
        c.go_to_named_pose("back", "a_bot")
        c.go_to_named_pose("screw_ready", "b_bot")
        c.go_to_named_pose("screw_ready", "c_bot")
      elif r == '611':
        c.screw_holder_tests(robot_name="b_bot")
      elif r == '612':
        c.screw_holder_tests(robot_name="c_bot")
      elif r == '621':
        c.go_to_named_pose("back", "c_bot")
        c.go_to_named_pose("home", "b_bot")
        c.do_change_tool_action("b_bot", equip=True, screw_size = 4)
      elif r == '622':
        c.go_to_named_pose("back", "c_bot")
        c.do_change_tool_action("b_bot", equip=False, screw_size = 4)
      elif r == '623':
        c.go_to_named_pose("back", "b_bot")
        c.go_to_named_pose("home", "c_bot")
        c.do_change_tool_action("c_bot", equip=True, screw_size = 4)
      elif r == '624':
        c.go_to_named_pose("back", "b_bot")
        c.go_to_named_pose("home", "c_bot")
        c.do_change_tool_action("c_bot", equip=False, screw_size = 4)
      elif r == '625':
        c.go_to_named_pose("back", "b_bot")
        c.go_to_named_pose("home", "c_bot")
        c.do_change_tool_action("c_bot", equip=True, screw_size = 3)
      elif r == '626':
        c.go_to_named_pose("back", "b_bot")
        c.go_to_named_pose("home", "c_bot")
        c.do_change_tool_action("c_bot", equip=False, screw_size = 3)
      elif r == '627':
        c.go_to_named_pose("back", "c_bot")
        c.go_to_named_pose("home", "b_bot")
        c.do_change_tool_action("b_bot", equip=True, screw_size = 50)
      elif r == '628':
        c.go_to_named_pose("back", "c_bot")
        c.do_change_tool_action("b_bot", equip=False, screw_size = 50)
      elif r == '63':
        c.screw_tool_test_assembly(robot_name="b_bot")
      elif r == '64':
        c.screw_tool_test_assembly(robot_name="c_bot")
      elif r == '641':
        c.screw_tool_test_assembly(robot_name="c_bot",tool_name="_nut_tool_m6_tip_link")
      elif r == '65':
        c.tray_screw_calibration(robot_name="b_bot", end_effector_link="b_bot_screw_tool_m4_tip_link")
      elif r == '66':
        c.tray_screw_calibration(robot_name="c_bot", end_effector_link="c_bot_screw_tool_m4_tip_link")
      elif r == '661':
        c.kitting_trays_screw_test(robot_name="c_bot", end_effector_link="c_bot_screw_tool_m4_tip_link")
      elif r == '671':
        c.screw_pickup_test(robot_name="b_bot")
      elif r == '681':
        c.place_screw_test(set_name = "set_1_")
      elif r == '6812':
        c.place_screw_test(set_name = "set_1_", screw_number = 2)
      elif r == '6813':
        c.place_screw_test(set_name = "set_1_", screw_number = 3)
      elif r == '682':
        c.place_screw_test(set_name = "set_2_")
      elif r == '683':
        c.place_screw_test(set_name = "set_3_")
      elif r == '691':
        c.screw_action_test(robot_name="b_bot")
      elif r == '692':
        c.screw_action_test(robot_name="c_bot")
      elif r == 'x':
        break
      else:
        rospy.loginfo("Could not read: " + r)
    rospy.loginfo("============ Exiting!")

  except rospy.ROSInterruptException:
    print "Something went wrong."
