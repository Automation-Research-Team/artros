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

# This file is based on the kinetic MoveIt tutorial for the Python movegroup interface.

import sys
import copy
import rospy
import tf_conversions
import actionlib

import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import std_msgs.msg
import robotiq_msgs.msg

import o2as_msgs
import o2as_msgs.msg
import o2as_msgs.srv

from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

def all_close(goal, actual, tolerance):
  """
  Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
  @param: goal       A list of floats, a Pose or a PoseStamped
  @param: actual     A list of floats, a Pose or a PoseStamped
  @param: tolerance  A float
  @returns: bool
  """
  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True

class O2ASBaseRoutines(object):
  """
  This class contains the common helper and convenience functions used in the routines.
  The common functions include the initialization of the services and actions,
  and shorthand functions for the most common actions.
  """
  def __init__(self):
    # super(O2ASBaseRoutines, self).__init__()
    
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('assembly_example', anonymous=False)

    self.robots = moveit_commander.RobotCommander()
    self.groups = {"a_bot":moveit_commander.MoveGroupCommander("a_bot"),
              "b_bot":moveit_commander.MoveGroupCommander("b_bot"),
              "c_bot":moveit_commander.MoveGroupCommander("c_bot")}
              # "front_bots":moveit_commander.MoveGroupCommander("front_bots"),
              # "all_bots":moveit_commander.MoveGroupCommander("all_bots") }
    self.gripper_action_clients = { "a_bot":actionlib.SimpleActionClient('/a_bot_gripper/gripper_action_controller', robotiq_msgs.msg.CModelCommandAction), 
                               "b_bot":actionlib.SimpleActionClient('/b_bot_gripper/gripper_action_controller', robotiq_msgs.msg.CModelCommandAction), 
                               "c_bot":actionlib.SimpleActionClient('/c_bot_gripper/gripper_action_controller', robotiq_msgs.msg.CModelCommandAction) }

    self.pick_client = actionlib.SimpleActionClient('/o2as_skills/pick', o2as_msgs.msg.pickAction)
    self.place_client = actionlib.SimpleActionClient('/o2as_skills/place', o2as_msgs.msg.placeAction)
    self.align_client = actionlib.SimpleActionClient('/o2as_skills/align', o2as_msgs.msg.alignAction)
    self.insert_client = actionlib.SimpleActionClient('/o2as_skills/insert', o2as_msgs.msg.insertAction)
    self.screw_client = actionlib.SimpleActionClient('/o2as_skills/screw', o2as_msgs.msg.screwAction)

    self.urscript_client = rospy.ServiceProxy('/o2as_skills/sendScriptToUR', o2as_msgs.srv.sendScriptToUR)
    self.goToNamedPose_client = rospy.ServiceProxy('/o2as_skills/goToNamedPose', o2as_msgs.srv.goToNamedPose)

    # self.pick_client.wait_for_server() # wait for the clients to connect
    # self.place_client.wait_for_server() 
    # self.align_client.wait_for_server() 
    # self.insert_client.wait_for_server() 
    # self.screw_client.wait_for_server() 
    rospy.sleep(.5)   # 
    rospy.loginfo("Finished initializing class")
    
  ############## ------ Internal functions (and convenience functions)

  def go_to_pose_goal(self, group_name, pose_goal_stamped, speed = 1.0, high_precision = False):
    group = self.groups[group_name]
    group.set_pose_target(pose_goal_stamped)
    rospy.loginfo("Setting velocity scaling to " + str(speed))
    group.set_max_velocity_scaling_factor(speed)

    if high_precision:
      group.set_goal_tolerance(.000001) 
      group.set_planning_time(10) 

    plan = group.go(wait=True)
    group.stop()
    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
    group.clear_pose_targets()
    
    # Reset the precision
    if high_precision:
      group.set_goal_tolerance(.0001) 
      group.set_planning_time(3) 

    current_pose = group.get_current_pose().pose
    return all_close(pose_goal_stamped.pose, current_pose, 0.01)

  def go_to_named_pose(self, pose_name, robot_name, speed = 1.0):
    # pose_name should be "home_a", "home_b" etc.
    self.groups[robot_name].set_named_target(pose_name)
    rospy.loginfo("Setting velocity scaling to " + str(speed))
    self.groups[robot_name].set_max_velocity_scaling_factor(speed)
    self.groups[robot_name].go(wait=True)
    self.groups[robot_name].stop()
    self.groups[robot_name].clear_pose_targets()
    return True

  def do_pick_action(self, robot_name, pose_stamped, tool_name = ""):
    # Call the pick action
    goal = o2as_msgs.msg.pickGoal()
    goal.robot_name = robot_name
    goal.item_pose = pose_stamped
    goal.tool_name = tool_name
    rospy.loginfo("Sending pick action goal")
    rospy.loginfo(goal)

    self.pick_client.send_goal(goal)
    rospy.loginfo("Waiting for result")
    self.pick_client.wait_for_result()
    rospy.loginfo("Getting result")
    return self.pick_client.get_result()

  def do_place_action(self, robot_name, pose_stamped, tool_name = ""):
    # Call the pick action
    goal = o2as_msgs.msg.placeGoal()
    goal.robot_name = robot_name
    goal.item_pose = pose_stamped
    goal.tool_name = tool_name
    rospy.loginfo("Sending place action goal")
    rospy.loginfo(goal)

    self.place_client.send_goal(goal)
    rospy.loginfo("Waiting for result")
    self.place_client.wait_for_result()
    rospy.loginfo("Getting result")
    return self.place_client.get_result()

  def do_insertion(self, robot_name):
    # Currently calls the UR service directly rather than the action of the skill_server
    req = o2as_msgs.srv.sendScriptToURRequest()
    req.robot_name = robot_name
    req.program_id = "insertion"
    res = self.urscript_client.call(req)
    return res.success


  ################ ----- Routines  
  ################ 
  ################ 
