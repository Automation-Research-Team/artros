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
#
# Author: Toshio Ueshiba
#
import rospy
import actionlib
import numpy as np
from actionlib_msgs.msg import GoalStatus
from geometry_msgs      import msg as gmsg
from aist_routines      import msg as amsg
from tf                 import transformations as tfs

######################################################################
#  class SweepAction                                                 #
######################################################################
class SweepAction(object):
    def __init__(self, routines):
        super(SweepAction, self).__init__()

        self._routines = routines
        self._server   = actionlib.SimpleActionServer("sweep",
                                                      amsg.sweepAction,
                                                      self._execute_cb, False)
        self._server.register_preempt_callback(self._preempt_callback)
        self._server.start()
        self._client = actionlib.SimpleActionClient("sweep",
                                                    amsg.sweepAction)
        self._client.wait_for_server()

    # Client stuffs
    def execute(self, robot_name, pose_stamped, contact_offset, sweep_offset,
                approach_offset, departure_offset, speed_fast, speed_slow,
                interactive, wait=True, feedback_cb=None):
        goal = amsg.sweepGoal()
        goal.robot_name       = robot_name
        goal.pose             = pose_stamped
        goal.contact_offset   = self._create_transform(contact_offset)
        goal.sweep_offset     = self._create_transform(sweep_offset)
        goal.approach_offset  = self._create_transform(approach_offset)
        goal.departure_offset = self._create_transform(departure_offset)
        goal.speed_fast       = speed_fast
        goal.speed_slow       = speed_slow
        goal.interactive      = interactive
        self._client.send_goal(goal, feedback_cb=feedback_cb)
        if wait:
            return self.wait_for_result()
        else:
            return None

    def wait_for_result(self, timeout=rospy.Duration(0)):
        if self._client.wait_for_result(timeout):
            return self._client.get_result().result
        else:
            return None

    def cancel(self):
        if self._client.get_state() in ( GoalStatus.PENDING,
                                         GoalStatus.ACTIVE ):
            self._client.cancel_goal()

    # Server stuffs
    def shutdown(self):
        self._server.__del__()

    def _create_transform(self, offset):
        xyz = (0, 0, 0)    if len(offset) < 3 else offset[0:3]
        q   = (0, 0, 0, 1) if len(offset) < 6 else \
              tfs.quaternion_from_euler(
                  *np.radians(offset[3:6])) if len(offset) == 6 else \
              offset[3:7]
        return gmsg.Transform(gmsg.Vector3(*xyz), gmsg.Quaternion(*q))

    def _execute_cb(self, goal):
        rospy.loginfo("*** Do sweeping ***")
        routines = self._routines
        gripper  = routines.gripper(goal.robot_name)
        result   = amsg.sweepResult()

        if goal.interactive:
            print('------------ Select sweep direction -------------')
            print('  X: positive direction along x-axis')
            print('  x: negative direction along x-axis')
            print('  Y: positive direction along y-axis')
            print('  y: negative direction along y-axis')
            key = raw_input('> ')
            if key == 'x':
                goal.contact_offset.translation.x *= -1
                goal.contact_offset.translation.y *= -1
                goal.sweep_offset.translation.x   *= -1
                goal.sweep_offset.translation.y   *= -1
            elif key == 'Y':
                x = goal.contact_offset.translation.x
                y = goal.contact_offset.translation.y
                goal.contact_offset.translation.x = -y
                goal.contact_offset.translation.y = x
                x = goal.sweep_offset.translation.x
                y = goal.sweep_offset.translation.y
                goal.sweep_offset.translation.x = -y
                goal.sweep_offset.translation.y = x
            elif key == 'y':
                x = goal.contact_offset.translation.x
                y = goal.contact_offset.translation.y
                goal.contact_offset.translation.x = y
                goal.contact_offset.translation.y = -x
                x = goal.sweep_offset.translation.x
                y = goal.sweep_offset.translation.y
                goal.sweep_offset.translation.x = y
                goal.sweep_offset.translation.y = -x
            else:
                pass

        # Go to approach pose.
        rospy.loginfo("--- Go to approach pose. ---")
        if not self._is_active(amsg.sweepFeedback.MOVING):
            return
        target_pose \
            = routines.effector_target_pose(goal.pose,
                                            (goal.approach_offset.translation.x,
                                             goal.approach_offset.translation.y,
                                             goal.approach_offset.translation.z,
                                             goal.approach_offset.rotation.x,
                                             goal.approach_offset.rotation.y,
                                             goal.approach_offset.rotation.z,
                                             goal.approach_offset.rotation.w))
        success, _, _ = routines.go_to_pose_goal(goal.robot_name, target_pose,
                                                 goal.speed_fast)
        if not success:
            result.result = amsg.sweepResult.MOVE_FAILURE
            self._server.set_aborted(result, "Failed to go to approach pose")
            return

        # Approach contact pose.
        rospy.loginfo("--- Go to contact pose. ---")
        if not self._is_active(amsg.sweepFeedback.APPROACHING):
            return
        target_pose \
            = routines.effector_target_pose(goal.pose,
                                            (goal.contact_offset.translation.x,
                                             goal.contact_offset.translation.y,
                                             goal.contact_offset.translation.z,
                                             goal.contact_offset.rotation.x,
                                             goal.contact_offset.rotation.y,
                                             goal.contact_offset.rotation.z,
                                             goal.contact_offset.rotation.w))
        routines.add_marker("pick_pose", target_pose)
        routines.publish_marker()
        success, _, _ = routines.go_to_pose_goal(goal.robot_name, target_pose,
                                                 goal.speed_slow)
        if not success:
            result.result = amsg.sweepResult.APPROACH_FAILURE
            self._server.set_aborted(result, "Failed to approach target")
            return

        # Sweep.
        rospy.loginfo("--- Sweep. ---")
        if not self._is_active(amsg.sweepFeedback.SWEEPING):
            return
        target_pose \
            = routines.effector_target_pose(goal.pose,
                                            (goal.sweep_offset.translation.x,
                                             goal.sweep_offset.translation.y,
                                             goal.sweep_offset.translation.z,
                                             goal.sweep_offset.rotation.x,
                                             goal.sweep_offset.rotation.y,
                                             goal.sweep_offset.rotation.z,
                                             goal.sweep_offset.rotation.w))
        routines.add_marker("place_pose", target_pose)
        routines.publish_marker()
        success, _, _ = routines.go_to_pose_goal(goal.robot_name, target_pose,
                                                 goal.speed_slow)
        if not success:
            result.result = amsg.sweepResult.SWEEP_FAILURE
            self._server.set_aborted(result, "Failed to sweep")
            return

        # Go back to departure(pick) or approach(place) pose.
        rospy.loginfo("--- Go back to departure pose. ---")

        if not self._is_active(amsg.sweepFeedback.DEPARTING):
            return
        success, _, _ = routines.go_to_pose_goal(
                             goal.robot_name,
                             routines.effector_target_pose(
                                 goal.pose,
                                 (goal.departure_offset.translation.x,
                                  goal.departure_offset.translation.y,
                                  goal.departure_offset.translation.z,
                                  goal.departure_offset.rotation.x,
                                  goal.departure_offset.rotation.y,
                                  goal.departure_offset.rotation.z,
                                  goal.departure_offset.rotation.w)),
                             goal.speed_fast)
        if not success:
            result.result = amsg.sweepResult.DEPARTURE_FAILURE
            self._server.set_aborted(result, "Failed to depart from target")
            return

        if success:
            result.result = amsg.sweepResult.SUCCESS
            self._server.set_succeeded(result, "Succeeded")
        else:
            result.result = amsg.sweepResult.GRASP_FAILURE
            self._server.set_aborted(result, "Failed to grasp")

    def _preempt_callback(self):
        robot_name = self._server.current_goal.get_goal().robot_name
        self._routines.stop(robot_name)
        self._routines.gripper(robot_name).cancel()
        self._server.set_preempted()

    def _is_active(self, state):
        if self._server.is_active():
            self._server.publish_feedback(amsg.sweepFeedback(state=state))
            return True
        return False