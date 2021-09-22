# Software License Agreement (BSD License)
#
# Copyright (c) 2021, National Institute of Advanced Industrial Science and Technology (AIST)
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
#  * Neither the name of National Institute of Advanced Industrial
#    Science and Technology (AIST) nor the names of its contributors
#    may be used to endorse or promote products derived from this software
#    without specific prior written permission.
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

import rospy
import actionlib

from control_msgs.msg    import (FollowJointTrajectoryAction,
                                 FollowJointTrajectoryGoal,
                                 PointHeadAction, PointHeadGoal)
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

######################################################################
#  class FollowTrajectoryClinet                                      #
######################################################################
class FollowTrajectoryClient(object):
    """
    Send a trajectory to controller
    """

    def __init__(self, name, joint_names):
        self._client = actionlib.SimpleActionClient(
            "%s/follow_joint_trajectory" % name,
            FollowJointTrajectoryAction)
        rospy.loginfo("Waiting for %s..." % name)
        self._client.wait_for_server()
        self._joint_names = joint_names

    def move_to(self, positions, duration):
        if len(self._joint_names) != len(positions):
            rospy.logerr("Invalid trajectory position")
            return False

        trajectory = JointTrajectory()
        trajectory.joint_names = self._joint_names
        trajectory.points.append(JointTrajectoryPoint())
        trajectory.points[0].positions       = positions
        trajectory.points[0].velocities      = [0.0 for _ in positions]
        trajectory.points[0].accelerations   = [0.0 for _ in positions]
        trajectory.points[0].time_from_start = rospy.Duration(duration)
        goal = FollowJointTrajectoryGoal()
        goal.trajectory = trajectory
        return self._client.send_goal_and_wait(goal)

    def follow(self, positions_list, duration):
        time_step  = duration/len(positions_list)
        trajectory = JointTrajectory()
        trajectory.joint_names = self._joint_names
        for i, positions in enumerate(positions_list):
            trajectory.points.append(JointTrajectoryPoint())
            trajectory.points[i].positions       = positions
            trajectory.points[i].velocities      = [0.0 for _ in positions]
            trajectory.points[i].accelerations   = [0.0 for _ in positions]
            trajectory.points[i].time_from_start = rospy.Duration(
                                                        (i + 1)*time_step)
        goal = FollowJointTrajectoryGoal()
        goal.trajectory = trajectory
        return self._client.send_goal_and_wait(goal)
