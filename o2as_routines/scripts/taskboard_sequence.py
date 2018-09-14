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
import tf_conversions
from math import pi
from math import *

from o2as_msgs.srv import *
import actionlib
import o2as_msgs.msg

import o2as_msgs
import o2as_msgs.srv


from o2as_routines.base import O2ASBaseRoutines

class TaskboardClass(O2ASBaseRoutines):
  """
  This contains the routine used to run the taskboard task.
  """
  def __init__(self):
    super(TaskboardClass, self).__init__()
    self.set_up_item_parameters()
    
    # self.action_client.wait_for_server()
    rospy.sleep(.5)   # Use this instead of waiting, so that simulation can be used

  def set_up_item_parameters(self):
    # These parameters should probably be read from a csv file.
    self.item_names = ["Bearing with housing", "6 mm bearing retainer pin", "17 mm spacer for bearings", 
                      "9 mm spacer for bearings", "Rotary shaft", "4 mm round belt", 
                      "M6 Nut & Bolt", "M12 nut", "6 mm washer", 
                      "10 mm washer", "M3 set screw", "M3 bolt", 
                      "M4 bolt", "Pulley", "10 mm end cap"]
    self.item_pick_heights = [0.02, 0.02, 0.025, 
                              0.047, 0.02, 0.0, 
                              0.02, 0.02, -0.005, 
                              -0.005, 0.02, 0.02,
                              0.02, 0.007, -0.005]

    self.item_place_heights = [0.04, 0.04, 0.035,
                               0.046, 0.04, 0.04, 
                               0.04, 0.04, 0.002, 
                               0.002, 0.04, 0.04, 
                               0.04, 0.006, 0.0]
    self.gripper_operation_to_use = ["outer", "inner_from_inside", "inner_from_outside", "complex_pick_from_inside", "complex_pick_from_outside"]
    downward_orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, 0))
    # 
    self.pick_poses = []
    self.place_poses = []
    for i in range(0,15):
      pick_pose = geometry_msgs.msg.PoseStamped()
      pick_pose.pose.orientation = downward_orientation
      pick_pose.header.frame_id = "mat_part" + str(i+1)
      pick_pose.pose.position.z = self.item_pick_heights[i]
      self.pick_poses.append(pick_pose)

      place_pose = geometry_msgs.msg.PoseStamped()
      place_pose.pose.orientation = downward_orientation
      place_pose.header.frame_id = "taskboard_part" + str(i+1)
      # 
      place_pose.pose.position.z = self.item_place_heights[i]
      self.place_poses.append(place_pose)
    

  # def 
  # calib_pose = geometry_msgs.msg.PoseStamped()
  #   #calib_pose.header.frame_id = "taskboard_part4"
  #   calib_pose.header.frame_id = "mat_part4"
  #   calib_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, pi/2))
  #   rospy.loginfo(calib_pose.pose.orientation)
  #   calib_pose.pose.position.x = 0
  #   calib_pose.pose.position.z = 0.15

  #   self.go_to_named_pose("home", "a_bot")
  #   self.go_to_named_pose("home", "b_bot")
  #   self.go_to_named_pose("home", "c_bot")

  #   self.go_to_pose_goal("a_bot", calib_pose, speed=0.3)
  #   print "============ Press `Enter` to move a_bot ..."
  #   raw_input()
  #   calib_pose.pose.position.z -= (.12)
  #   self.go_to_pose_goal("a_bot", calib_pose, speed=0.02)

  #   print "============ Press `Enter` to move a_bot ..."
  #   raw_input()
  #   calib_pose.pose.position.z += (.12)
  #   self.go_to_pose_goal("a_bot", calib_pose, speed=0.02)
  #   print "============ Press `Enter` to move ..."
  #   raw_input()
  #   self.go_to_named_pose("home", "a_bot",speed=0.3)
    

  ################ ----- Routines  
  ################ 
  ################ 
  def pick(self, robotname, object_pose, grasp_height, speed_fast, speed_slow, gripper_command, approach_height = 0.05):
    self.publish_marker(object_pose, "pick_pose")
    #initial gripper_setup
    rospy.loginfo("Going above object to pick")
    object_pose.pose.position.z += approach_height
    self.go_to_pose_goal(robotname, object_pose, speed=speed_fast)
    object_pose.pose.position.z -= approach_height

    if gripper_command=="complex_pick_from_inside":
      self.precision_gripper_inner_close() 
    elif gripper_command=="complex_pick_from_outside":
      self.precision_gripper_inner_open()
    elif gripper_command=="easy_pick_only_inner":
      self.precision_gripper_inner_close()
    else: 
      self.send_gripper_command(gripper=robotname, command="open")

    rospy.loginfo("Moving down to object")
    rospy.loginfo(grasp_height)
    object_pose.pose.position.z += grasp_height
    self.go_to_pose_goal(robotname, object_pose, speed=speed_slow, high_precision=True)
    object_pose.pose.position.z -= grasp_height

    # W = raw_input("waiting for the gripper")
    #gripper close
    if gripper_command=="complex_pick_from_inside":
      self.precision_gripper_inner_open(this_action_grasps_an_object = True)
      self.precision_gripper_outer_close()
    elif gripper_command=="complex_pick_from_outside":
      self.precision_gripper_inner_close(this_action_grasps_an_object = True)
      self.precision_gripper_outer_close()
    elif gripper_command=="easy_pick_only_inner":
      self.precision_gripper_inner_open(this_action_grasps_an_object = True)
    else: 
      self.send_gripper_command(gripper=robotname, command="close")
    rospy.sleep(.5)
    rospy.loginfo("Going back up")
    object_pose.pose.position.z += approach_height
    self.go_to_pose_goal(robotname, object_pose, speed=speed_fast)
    object_pose.pose.position.z -= approach_height

######

  def place(self,robotname, object_pose, place_height, speed_fast, speed_slow, gripper_command, approach_height = 0.05, lift_up_after_place = True):
    self.publish_marker(object_pose, "place_pose")
    rospy.loginfo("Going above place target")
    object_pose.pose.position.z += approach_height
    self.go_to_pose_goal(robotname, object_pose, speed=speed_fast)
    object_pose.pose.position.z -= approach_height

    rospy.loginfo("Moving to place target")
    object_pose.pose.position.z += place_height
    self.go_to_pose_goal(robotname, object_pose, speed=speed_slow, high_precision=True)
    object_pose.pose.position.z -= place_height

    # print "============ Stopping at the placement height. Press `Enter` to keep moving moving the robot ..."
    # raw_input()

    #gripper open
    if gripper_command=="complex_pick_from_inside":
      self.precision_gripper_outer_open()
      self.precision_gripper_inner_close()
    elif gripper_command=="complex_pick_from_outside":
      self.precision_gripper_outer_open()
      self.precision_gripper_inner_open()
    elif gripper_command=="easy_pick_only_inner":
      self.precision_gripper_inner_close()
    else: 
      self.send_gripper_command(gripper=robotname, command="open")

    
    if lift_up_after_place:
      rospy.loginfo("Moving back up")
      object_pose.pose.position.z += approach_height
      self.go_to_pose_goal(robotname, object_pose, speed=speed_fast)  
      object_pose.pose.position.z -= approach_height
    
  def belt_circle_motion(self, robot_name, speed = 0.02):
    self.toggle_collisions(collisions_on=False)
    group = self.groups[robot_name]
    rospy.loginfo("Performing belt spiral motion " + str(speed))
    rospy.loginfo("Setting velocity scaling to " + str(speed))
    group.set_max_velocity_scaling_factor(speed)

    r_pulley=0.034
    theta_offset = 90  # To adjust the starting angle
    theta_belt= 0 + theta_offset
    theta_increase=15

    start_pose = geometry_msgs.msg.PoseStamped()
    start_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, 0))
    start_pose.header.frame_id = "taskboard_part6_large_pulley"
    start_pose.pose.position.z = 0.07
    start_pose.pose.position.x = cos(radians(theta_belt))*r_pulley
    start_pose.pose.position.y = sin(radians(theta_belt))*r_pulley
    self.go_to_pose_goal(robot_name, start_pose, speed=0.5)

    start_pose.pose.position.z = 0
    self.go_to_pose_goal(robot_name, start_pose, speed=0.02)
    
    next_pose = geometry_msgs.msg.PoseStamped()
    next_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, 0))
    next_pose.header.frame_id = "taskboard_part6_large_pulley"
    while theta_belt <= 360+theta_offset and not rospy.is_shutdown():
        #By default, the Spiral_Search function will maintain contact between both mating parts at all times
         theta_belt=theta_belt+theta_increase
         x=cos(radians(theta_belt))*r_pulley
         y=sin(radians(theta_belt))*r_pulley
         next_pose.pose.position.x = x
         next_pose.pose.position.y = y
         print(theta_belt)
        #  print(radians(theta_belt))
         print(cos(radians(theta_belt)))
         print(cos(radians(theta_belt))*r_pulley)
         print(next_pose.pose.position)
         self.go_to_pose_goal(robot_name, next_pose)
         rospy.sleep(0.1)
    
    self.toggle_collisions(collisions_on=True)
    # -------------
    return True
  
  ####

  def full_taskboard_task(self):
    self.groups["a_bot"].set_goal_tolerance(.0001) 
    self.groups["a_bot"].set_planning_time(5) 
    self.groups["a_bot"].set_num_planning_attempts(1000) 
    self.go_to_named_pose("home", "c_bot")
    self.go_to_named_pose("home", "b_bot")
    self.go_to_named_pose("home", "a_bot")
  
    for i in range(0,15):
      rospy.loginfo("=== Now targeting part number " + str(i+1) + ": " + self.item_names[i])

      # peg-in-hole with complex_pick_from_inside
      if i in [2, 13]:
        self.pick("a_bot",self.pick_poses[i],self.item_pick_heights[i],
                    speed_fast = 0.2, speed_slow = 0.02, gripper_command="complex_pick_from_inside")
        self.place("a_bot",self.place_poses[i],self.item_place_heights[i],
                    speed_fast = 0.2, speed_slow = 0.02, gripper_command="complex_pick_from_inside")
      
      #peg-in-hole with complex_pick_from_outside
      if i == 3:
        self.pick("a_bot",self.pick_poses[i],self.item_pick_heights[i],
                    speed_fast = 0.2, speed_slow = 0.02, gripper_command="complex_pick_from_outside")
        self.place("a_bot",self.place_poses[i],self.item_place_heights[i],
                    speed_fast = 0.2, speed_slow = 0.02, gripper_command="complex_pick_from_outside")

      #peg-in-hole with easy pick_only_inner(washers)
      if i in [8, 9, 14]:
        self.pick("a_bot",self.pick_poses[i],self.item_pick_heights[i],
                    speed_fast = 0.2, speed_slow = 0.02, gripper_command="easy_pick_only_inner")
        self.place("a_bot",self.place_poses[i],self.item_place_heights[i],
                    speed_fast = 0.2, speed_slow = 0.02, gripper_command="easy_pick_only_inner")

      # This requires a regrasp (bearing)
      if i == 0:
        rospy.logwarn("Now it's part1. It's a regrasp task")
        # self.go_to_pose_goal("b_bot", pick_poses[i] speed=speed_fast)

      # Requires a regrasp (pin)
      if i == 1:
        rospy.logwarn("This part is skipped because it requires a regrasp")
        pass

      # Screwing
      # if i in [4, 7, 10, 11, 12]:
      #   self.pick("a_bot",self.pick_poses[i],self.item_pick_heights[i], speed_fast = 0.2, speed_slow = 0.02)
      #   self.place("a_bot",self.place_poses[i],self.item_place_heights[i], speed_fast = 0.2, speed_slow = 0.02)
      
      # Requires multiple robots (Bolt and nut)
      if i == 6:
        rospy.logwarn("This part is skipped because it requires multiple robots interacting")
        pass

      # Requires special strategy (Belt)
      if i == 5:
        rospy.logwarn("This part is skipped because it requires a special strategy")
        pass


      self.go_to_named_pose("home", "c_bot")
      self.go_to_named_pose("home", "b_bot")
      self.go_to_named_pose("home", "a_bot")

  def taskboard_manual_testing(self):
    self.groups["a_bot"].set_goal_tolerance(.0001) 
    self.groups["a_bot"].set_planning_time(5) 
    self.go_to_named_pose("home", "c_bot")
    self.go_to_named_pose("home", "b_bot")
    self.go_to_named_pose("home", "a_bot")
    
    downward_orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, 0))

    # Perform the pick/place operations
    i = raw_input("the number of the part")
    rospy.loginfo("=== Now targeting part number " + str(i+1) + ": " + self.item_names[i])
    
    self.groups["a_bot"].set_goal_tolerance(.000001) 
    self.groups["a_bot"].set_planning_time(10) 
    self.groups["a_bot"].set_num_planning_attempts(1000) 
    if i == 3:  
      self.do_pick_action("a_bot", pick_poses[i], tool_name = "", do_complex_pick_from_inside=True)
      self.do_place_action("a_bot", place_poses[i], tool_name = "")
      pass
    return

  def kitting_test(self,robotname,object_pose):
    self.groups["a_bot"].set_goal_tolerance(.0001) 
    self.groups["a_bot"].set_planning_time(5) 
    self.go_to_named_pose("home", "a_bot")
    downward_orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, 0))
    picking_pose = geometry_msgs.msg.PoseStamped()
    picking_pose.pose.orientation = downward_orientation
    picking_pose.header.frame_id = object_pose
    picking_pose.pose.position.z = 0.15
    
    self.go_to_pose_goal(robotname, picking_pose)

  def belt_pick(self, robotname):
    belt_pick_pose = copy.deepcopy(self.pick_poses[5])
    belt_pick_pose.pose.position.y += .055
    self.pick("b_bot", belt_pick_pose, grasp_height=.002,
                    speed_fast = 0.2, speed_slow = 0.02, gripper_command="close")

  def precision(self, robotname, object_pose, grasp_height, object_place, place_height, approach_height = 0.05, speed_fast=0.2, speed_slow=0.02):
    #grasp

    rospy.loginfo("Going above object to pick")
    object_pose.pose.position.z += approach_height
    self.go_to_pose_goal(robotname, object_pose, speed=speed_fast)
    object_pose.pose.position.z -= approach_height

    rospy.loginfo("Moving down to object")
    rospy.loginfo(grasp_height)
    object_pose.pose.position.z = grasp_height
    self.go_to_pose_goal(robotname, object_pose, speed=speed_slow, high_precision=True)
 

    W = raw_input("waiting for the gripper")
    self.precision_gripper_outer_close()
    self.precision_gripper_inner_close()

    
    rospy.loginfo("Going back up")
    object_pose.pose.position.z += 0.2
    self.go_to_pose_goal(robotname, object_pose, speed=speed_fast)
    object_pose.pose.position.z -= 0.2
########################################
    #place approach

    #"Moving to place target

    object_place.pose.position.z += approach_height
    self.go_to_pose_goal(robotname, object_place, speed=speed_fast)  
    object_place.pose.position.z -= approach_height

    rospy.loginfo("Moving to place target")
    object_place.pose.position.z = place_height
    self.go_to_pose_goal(robotname, object_place, speed=speed_slow, high_precision=True)


    print "============ Stopping at the placement height. Press `Enter` to keep moving moving the robot ..."
    raw_input()
    self.precision_gripper_outer_open()
    self.precision_gripper_inner_open()

    rospy.loginfo("Moving back up")
    object_place.pose.position.z += 0.2
    self.go_to_pose_goal(robotname, object_place, speed=speed_fast)  
    object_place.pose.position.z -= 0.2



if __name__ == '__main__':
  try:
    taskboard = TaskboardClass()
    taskboard.set_up_item_parameters()
    #taskboard.full_taskboard_task()
    
    # 3,14        complex_pick_from_inside
    # 4           complex_pick_from_outside
    # 9, 10, 15   easy_pick_only_inner

    taskboard.groups["a_bot"].set_goal_tolerance(.0001) 
    taskboard.groups["a_bot"].set_planning_time(3) 
    taskboard.groups["a_bot"].set_num_planning_attempts(10)
    taskboard.go_to_named_pose("home", "c_bot")
    taskboard.go_to_named_pose("home", "b_bot")
    taskboard.go_to_named_pose("home", "a_bot")

    # taskboard.full_taskboard_task()

    i = raw_input("Enter the number of the part to be performed: ")
    i =int(i)
    while(i):
      if i == 111:
        #taskboard.precision("a_bot", taskboard.pick_poses[3], taskboard.item_pick_heights[3]-0.012, taskboard.place_poses[3], taskboard.item_place_heights[3]-0.02)
        taskboard.precision("a_bot", taskboard.pick_poses[3], taskboard.item_pick_heights[3]-0.026, taskboard.place_poses[3], taskboard.item_place_heights[3]-0.025)
      # Testing
      if i == 50:
        taskboard.go_to_pose_goal("a_bot",  taskboard.pick_poses[0], "b_bot", taskboard.pick_poses[2])
      if i == 51:
        taskboard.move_front_bots(taskboard.place_poses[10], taskboard.pick_poses[0], speed=0.04)
        rospy.loginfo("Moved robots to first pose. Press enter to move to the next")
        raw_input()
        taskboard.move_front_bots(taskboard.pick_poses[0], taskboard.place_poses[10], speed=0.04)
        rospy.loginfo("Waiting for enter before going home")
        raw_input()

      if i == 21:
        taskboard.belt_circle_motion("a_bot")

      if i == 1:
        #taskboard.go_to_pose_goal("b_bot", taskboard.pick_poses[i-1], speed = 0.2)

        taskboard.do_pick_action("b_bot", taskboard.pick_poses[i-1], z_axis_rotation = 0.0, use_complex_planning = False)
        taskboard.do_regrasp("b_bot", "a_bot", grasp_distance = .03)
        taskboard.place("a_bot", taskboard.place_poses[i-1],taskboard.item_place_heights[i-1],
                                speed_fast = 0.2, speed_slow = 0.02, gripper_command="complex_pick_from_inside",
                                approach_height = 0.10, lift_up_after_place = False)
        taskboard.horizontal_spiral_motion("a_bot", .006, taskboard.place_poses[i-1])

      if i == 20:
        rospy.loginfo("doing spiral motion")
        taskboard.horizontal_spiral_motion("a_bot", .05)
      
      if i == 2:
        pass

      if i == 3:
        taskboard.pick("a_bot",taskboard.pick_poses[i-1],taskboard.item_pick_heights[i-1],
                                speed_fast = 0.2, speed_slow = 0.02, gripper_command="easy_pick_only_inner",
                                approach_height = 0.1)
        taskboard.place("a_bot",taskboard.place_poses[i-1],taskboard.item_place_heights[i-1],
                                speed_fast = 0.2, speed_slow = 0.02, gripper_command="easy_pick_only_inner",
                                approach_height = 0.15, lift_up_after_place = False)
        taskboard.horizontal_spiral_motion("a_bot", .004)
        rospy.loginfo("doing spiral motion")
      
      if i == 4:
        taskboard.pick("a_bot",taskboard.pick_poses[i-1],taskboard.item_pick_heights[i-1],
                                speed_fast = 0.2, speed_slow = 0.02, gripper_command="easy_pick_only_inner",
                                approach_height = 0.1)
        taskboard.place("a_bot",taskboard.place_poses[i-1],taskboard.item_place_heights[i-1],
                                speed_fast = 0.2, speed_slow = 0.02, gripper_command="easy_pick_only_inner",
                                approach_height = 0.15, lift_up_after_place = False)
        taskboard.horizontal_spiral_motion("a_bot", .002)

      if i == 6:
        # Pick up the belt
        taskboard.toggle_collisions(collisions_on=False)
        taskboard.belt_pick("b_bot")
        taskboard.toggle_collisions(collisions_on=True)
        taskboard.go_to_named_pose("home", "b_bot")
        
        # Set the placement aid
        pick_tool_pose = geometry_msgs.msg.PoseStamped()
        pick_tool_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, 0))
        pick_tool_pose.header.frame_id = "belt_placement_tool"
        taskboard.pick("c_bot", pick_tool_pose, grasp_height=0.013, speed_fast = 0.2, speed_slow = 0.1, gripper_command="close",
                                approach_height = 0.05)
        place_tool_pose = geometry_msgs.msg.PoseStamped()
        place_tool_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, 0))
        place_tool_pose.header.frame_id = "belt_placement_tool"
        place_tool_pose.pose.position.x = 0.058
        place_tool_pose.pose.position.z = 0.04
        taskboard.toggle_collisions(collisions_on=False)
        taskboard.place("c_bot", place_tool_pose, place_height=-0.005, speed_fast = 0.2, speed_slow = 0.03, gripper_command="open",
                                approach_height = 0.03, lift_up_after_place = True)
        taskboard.toggle_collisions(collisions_on=True)

        # Place the belt
        taskboard.go_to_named_pose("back", "c_bot")
        belt_place_pose = geometry_msgs.msg.PoseStamped()
        belt_place_pose.header.frame_id = "taskboard_part6_large_pulley"
        belt_place_pose.pose.position.x = 0.0
        belt_place_pose.pose.position.y = .068
        belt_place_pose.pose.position.z = .06 + .1
        belt_place_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, -170.0 * pi/180))
        # 0.716; -0.09; -0.684; 0.1
        taskboard.go_to_pose_goal("b_bot", belt_place_pose, speed=1.0)
        belt_place_pose.pose.position.z = .06
        taskboard.go_to_pose_goal("b_bot", belt_place_pose, speed=0.3)

        # In large pulley frame
        belt_place_pose.header.frame_id = "taskboard_part6_large_pulley"
        belt_place_pose.pose.position.x = 0.0
        belt_place_pose.pose.position.y = .01
        belt_place_pose.pose.position.z = .0075
        belt_place_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, -179.0 * pi/180))
        taskboard.go_to_pose_goal("b_bot", belt_place_pose, speed=0.1)
        
        taskboard.send_gripper_command(gripper="b_bot", command=.01)
        rospy.sleep(1)
        taskboard.send_gripper_command(gripper="b_bot", command="open")
        belt_place_pose.pose.position.z += .02
        taskboard.go_to_pose_goal("b_bot", belt_place_pose, speed=1.0)
        rospy.sleep(1)
        taskboard.go_to_named_pose("home", "b_bot", wait=True)

        # Fiddle in the belt
        rospy.logwarn("Doing belt spiral motion")
        taskboard.belt_circle_motion("a_bot")



      #15 is not adjusted yet  
      if i in [9, 10]:
        taskboard.pick("a_bot",taskboard.pick_poses[i-1],taskboard.item_pick_heights[i-1], approach_height = 0.05,
                                speed_fast = 0.2, speed_slow = 0.02, gripper_command="easy_pick_only_inner")
        taskboard.place("a_bot",taskboard.place_poses[i-1],taskboard.item_place_heights[i-1], approach_height = 0.05,
                                speed_fast = 0.2, speed_slow = 0.02, gripper_command="easy_pick_only_inner",
                                lift_up_after_place = False)
        if i == 9:
          taskboard.horizontal_spiral_motion("a_bot", .0025)
        if i == 10:
          taskboard.horizontal_spiral_motion("a_bot", .003)

      if i == 14:
        taskboard.pick("a_bot",taskboard.pick_poses[i-1],taskboard.item_pick_heights[i-1], approach_height = 0.05,
                                speed_fast = 0.2, speed_slow = 0.02, gripper_command="easy_pick_only_inner")
        taskboard.place("a_bot",taskboard.place_poses[i-1],taskboard.item_place_heights[i-1], approach_height = 0.05,
                                speed_fast = 0.2, speed_slow = 0.02, gripper_command="easy_pick_only_inner",
                                lift_up_after_place = False)
        taskboard.horizontal_spiral_motion("a_bot", .002)
        
      
      taskboard.go_to_named_pose("home", "a_bot")
      taskboard.go_to_named_pose("home", "b_bot")
      taskboard.go_to_named_pose("home", "c_bot")
      i = raw_input("Enter the number of the part to be performed: ")
      i =int(i)
    print "============ Done!"
  except rospy.ROSInterruptException:
    pass
