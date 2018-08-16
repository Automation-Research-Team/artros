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
import o2as_skills
import o2as_skills.msg
import o2as_skills.srv
from o2as_msgs.srv import *

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

class ExampleClass(object):
  """ExampleClass"""
  
  def insertion_demo(self):
    # Pick a thing with b_bot, then c_bot, then insert one into the other
    # WORK IN PROGRESS! Poses are too close, robot configuration too close to singularity.
    
    # # Define the pose of each item
    # peg_pose = geometry_msgs.msg.PoseStamped()
    # peg_pose.header.frame_id = "workspace_center"
    # peg_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, 0))
    # peg_pose.pose.position.x = -0.2
    # peg_pose.pose.position.z = 0.1
    
    # bearing_pose = copy.deepcopy(peg_pose)
    # bearing_pose.pose.position.x = 0.0
    # bearing_pose.pose.position.y = 0.15
    # bearing_pose.pose.position.z = 0.06
    
    # # First pick up item with b_bot
    # self.do_pick_action("b_bot", bearing_pose, tool_name = "")
    # self.go_to_named_pose("home_b", "b_bot")
    
    # # Now pick the item with c_bot
    # self.do_pick_action("c_bot", peg_pose, tool_name = "")
    # self.go_to_named_pose("home_c", "c_bot")

    # # Move the robots to a pose and tell one to insert
    # pre_insertion_pose_c = geometry_msgs.msg.PoseStamped()
    # pre_insertion_pose_c.header.frame_id = "workspace_center"
    # pre_insertion_pose_c.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, 0, pi*1/4))
    # # pre_insertion_pose_c.pose.orientation = geometry_msgs.msg.Quaternion(*[0.0024936, 0.023435, 0.40201, 0.91533])
    # pre_insertion_pose_c.pose.position.x = -0.21
    # pre_insertion_pose_c.pose.position.y = 0.18
    # pre_insertion_pose_c.pose.position.z = 0.55

    # pre_insertion_pose_b = geometry_msgs.msg.PoseStamped()
    # pre_insertion_pose_b.header.frame_id = "workspace_center"
    # pre_insertion_pose_b.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, 0, -pi*3/4))
    # pre_insertion_pose_b.pose.position.x = -0.11 + .01/1.41
    # pre_insertion_pose_b.pose.position.y = 0.28 - .01/1.41
    # pre_insertion_pose_b.pose.position.z = 0.55
    
    # rospy.loginfo("Going to pre_insertion c")
    # self.go_to_pose_goal("c_bot", pre_insertion_pose_c)
    # rospy.loginfo("Going to pre_insertion b")
    # self.go_to_pose_goal("b_bot", pre_insertion_pose_b)

    # rospy.loginfo("Attempt insertion?")
    # raw_input()
    self.do_insertion("c_bot")
    rospy.loginfo("Done.")


def main():
  try:
    tutorial = ExampleClass()

    # print "============ Press `Enter` to start assembly test ..."
    # raw_input()
    # tutorial.insertion_demo()
    # tutorial.pick_place_demo()
    # tutorial.simple_taskboard_demo()
    # tutorial.taskboard_pick()
    tutorial.groups["a_bot"].set_goal_tolerance(.000001)
    tutorial.groups["a_bot"].set_planning_time(10)
    tutorial.groups["a_bot"].set_num_planning_attempts(1000)
    tutorial.taskboard_with_pick_action()
    

    print "============ Demo complete!"
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()