#!/usr/bin/env python

import sys
import copy
import rospy
import geometry_msgs.msg
import tf_conversions
import tf
from math import pi
import numpy as np

from o2as_msgs.srv import *
import actionlib
import o2as_msgs.msg
from o2as_usb_relay.srv import *

from o2as_routines.base import O2ASBaseRoutines

class KittingClass(O2ASBaseRoutines):
  """
  This contains the routine used to run the kitting task. See base.py for shared convenience functions.
  """
  def __init__(self):
    super(KittingClass, self).__init__()
    self.set_up_item_parameters()
    self.set_up_end_effector()
    self.set_up_place_position()
    rospy.sleep(.5)

  def set_up_item_parameters(self):
    self.item_names = []
    downward_orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, 0))
    # 
    
  def set_up_end_effector(self):
    self.suction = rospy.ServiceProxy("o2as_usb_relay_server/set_power", SetPower)

  def set_up_place_position(self):
    self.part_id = ["part_" + str(i) for i in range(4,17,1)]
    self.place_id = {
      "part_4" : "set3_tray_1_partition_4",
      "part_5" : "set3_tray_2_partition_6",
      "part_6" : "set3_tray_1_partition_3",
      "part_7" : "set3_tray_1_partition_2",
      "part_8" : "set3_tray_2_partition_1",
      "part_9" : "set3_tray_2_partition_4",
      "part_10": "set3_tray_2_partition_7",
      "part_11": "set3_tray_1_partition_1",
      "part_12": "set3_tray_2_partition_3",
      "part_13": "set3_tray_1_partition_5",
      "part_14": "set3_tray_2_partition_2",
      "part_15": "set3_tray_2_partition_5",
      "part_16": "set3_tray_2_partition_8"
    }

    self.gripper_id = {
      "part_4" : "suction",
      "part_5" : "suction",
      "part_6" : "gripper",
      "part_7" : "suction",
      "part_8" : "suction",
      "part_9" : "gripper",
      "part_10": "gripper",
      "part_11": "suction",
      "part_12": "suction",
      "part_13": "suction",
      "part_14": "gripper",
      "part_15": "gripper",
      "part_16": "gripper",
      "part_17": "gripper",
      "part_18": "gripper"
    }

  ################ ----- Routines  
  ################ 
  ################ 

  def switch_suction(self, on=False):
    return self.suction(1, on)

  def pick(self, robot_name, gripper_name, object_id, object_pose, speed_fast, speed_slow, approach_height=0.03):
    
    if gripper_name=="suction":
      self.groups[robot_name].set_end_effector_link(robot_name + '_dual_suction_gripper_pad_link')

    self.publish_marker(object_pose, "aist_vision_result")
    rospy.loginfo("Going above object to pick")
    approach_pose = geometry_msgs.msg.PoseStamped()
    approach_pose = copy.deepcopy(object_pose)
    approach_pose.pose.position.z += approach_height
    approach_pose.pose.orientation.x = -0.5
    approach_pose.pose.orientation.y = 0.5
    approach_pose.pose.orientation.z = 0.5
    approach_pose.pose.orientation.w = 0.5
    self.go_to_pose_goal(robot_name, approach_pose, speed=speed_fast)

    rospy.loginfo("Moving down to object")
    self.go_to_pose_goal(robot_name, object_pose, speed=speed_slow, high_precision=True)
    
    rospy.loginfo("Picking up on suction")
    # self.switch_suction(True)
    rospy.sleep(2)

    rospy.loginfo("Going back up")
    self.go_to_pose_goal(robot_name, approach_pose, speed=speed_fast)

  def place(self, robot_name, gripper_name, object_id, place_height, speed_fast, speed_slow, approach_height=0.05):

    if object_id < 3 and object_id > 16:
      rospy.logerr("This object_id is wrong!!")
      return
    if gripper_name=="suction":
      self.groups[robot_name].set_end_effector_link(robot_name + '_dual_suction_gripper_pad_link')

    goal_pose = geometry_msgs.msg.PoseStamped()
    goal_pose.header.frame_id = self.place_id["part_" + object_id]
    goal_pose.pose.orientation.x = -0.5
    goal_pose.pose.orientation.y = 0.5
    goal_pose.pose.orientation.z = 0.5
    goal_pose.pose.orientation.w = 0.5
    goal_pose.pose.position.z = approach_height
    self.go_to_pose_goal(robot_name, goal_pose, speed=speed_fast)

    rospy.loginfo("Moving down to object")
    goal_pose.pose.position.z = place_height
    self.go_to_pose_goal(robot_name, goal_pose, speed=speed_slow, high_precision=True)
    rospy.loginfo("Place down from suction")
    # self.switch_suction(False)
    rospy.sleep(2)

    rospy.loginfo("Going back up")
    goal_pose.pose.position.z += approach_height
    self.go_to_pose_goal(robot_name, goal_pose, speed=speed_fast)

    self.go_to_named_pose("home", "c_bot")
    self.go_to_named_pose("home", "b_bot")
    self.go_to_named_pose("home", "a_bot")

  ################ ----- Demos  
  ################ 
  ################ 

  def check_accessibility_by_b_bot(self):
    object_pose = geometry_msgs.msg.PoseStamped()

    object_pose.header.frame_id = "set1_bin3_1"
    object_pose.pose.position.x = -0.1
    object_pose.pose.position.y = -0.05
    object_pose.pose.position.z = 0.01
    object_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(pi/4, pi/2, 0))
    rospy.loginfo("Picking up an demo object.")
    self.pick("b_bot", 4, object_pose, 1.0, 1.0)
    tray_id = "set3_tray_1_partition_2"
    rospy.loginfo("Place down an demo object on tray partition.")
    self.place("b_bot", 4, 0.005, 1.0, 1.0)

    object_pose.header.frame_id = "set2_bin1_5"
    object_pose.pose.position.x = 0.03
    object_pose.pose.position.y = -0.04
    object_pose.pose.position.z = 0.01
    object_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(-pi/4, pi/4, 0))
    rospy.loginfo("Picking up an demo object.")
    self.pick("b_bot", 16, object_pose, 1.0, 1.0)
    rospy.loginfo("Place down an demo object on tray partition.")
    self.place("b_bot", 16, 0.005, 1.0, 1.0)

    
    object_pose.header.frame_id = "set2_bin1_1"
    object_pose.pose.position.x = 0.03
    object_pose.pose.position.y = 0.04
    object_pose.pose.position.z = 0.01
    object_pose.pose.orientation.x = -0.5
    object_pose.pose.orientation.y = 0.5
    object_pose.pose.orientation.z = 0.5
    object_pose.pose.orientation.w = 0.5
    rospy.loginfo("Picking up an demo object.")
    self.pick("b_bot", object_pose, 1.0, 1.0)
    tray_id = "set3_tray_1_partition_2"
    rospy.loginfo("Place down an demo object on tray partition.")
    self.place("b_bot", tray_id, 0.005, 1.0, 1.0)

    object_pose.header.frame_id = "set1_bin2_1"
    object_pose.pose.position.x = -0.08
    object_pose.pose.position.y = 0.03
    object_pose.pose.position.z = 0.01
    rospy.loginfo("Picking up an demo object.")
    self.pick("b_bot", object_pose, 1.0, 1.0)
    tray_id = "set3_tray_1_partition_2"
    rospy.loginfo("Place down an demo object on tray partition.")
    self.place("b_bot", tray_id, 0.005, 1.0, 1.0)

  def pick_and_place_demo(self):

    robot_name = "b_bot"
    self.groups[robot_name].set_end_effector_link(robot_name + '_dual_suction_gripper_pad_link')

    speed_fast = 1.0
    speed_slow = 1.0

    bin_id = {
      "part_4": "set1_bin2_1",
      "part_8": "set1_bin2_2",
      "part_11": "set1_bin2_3",
      "part_13": "set1_bin2_4",
      "part_6": "set1_bin3_1",
      "part_9": "set2_bin1_1",
      "part_12": "set2_bin1_2",
      "part_16": "set2_bin1_3",
      "part_17": "set2_bin1_4",
      "part_18": "set2_bin1_5"
    }

    object_pose = geometry_msgs.msg.PoseStamped()
    object_pose.pose.position.z = 0.01
    object_pose.pose.orientation.x = -0.5
    object_pose.pose.orientation.y = 0.5
    object_pose.pose.orientation.z = 0.5
    object_pose.pose.orientation.w = 0.5

    intermediate_pose = geometry_msgs.msg.PoseStamped()
    intermediate_pose.header.frame_id = "workspace_center"
    intermediate_pose.pose.position.z = 0.3
    intermediate_pose.pose.orientation.x = -0.5
    intermediate_pose.pose.orientation.y = 0.5
    intermediate_pose.pose.orientation.z = 0.5
    intermediate_pose.pose.orientation.w = 0.5

    for set_num in range(1,4):
      item_list = rospy.get_param("/set_"+str(set_num))
      rospy.loginfo("set_"+str(set_num))
      for item in item_list:
        object_pose.header.frame_id = bin_id["part_" + item["id"]]
        if self.gripper_id["part_" + item["id"]] == "suction":
          self.go_to_pose_goal(robot_name, intermediate_pose, speed_fast)
          self.pick(robot_name, self.gripper_id["part_" + item["id"]], item["id"], object_pose, speed_fast, speed_slow)
          self.go_to_pose_goal(robot_name, intermediate_pose, speed_fast)
          self.place(robot_name, self.gripper_id["part_" + item["id"]], item["id"], 0.01, speed_fast, speed_slow)

  def kitting_task(self):
    self.go_to_named_pose("home", "c_bot")
    self.go_to_named_pose("home", "b_bot")
    self.go_to_named_pose("home", "a_bot")

    self.pick_and_place_demo()

    self.go_to_named_pose("home", "c_bot")
    self.go_to_named_pose("home", "b_bot")
    self.go_to_named_pose("home", "a_bot")

    # TODO

if __name__ == '__main__':
  try:
    kit = KittingClass()
    kit.set_up_item_parameters()
    
    kit.kitting_task()

    print "============ Done!"
  except rospy.ROSInterruptException:
    pass
