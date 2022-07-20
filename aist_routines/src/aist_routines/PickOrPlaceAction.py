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
from geometry_msgs.msg  import Transform, Vector3, Quaternion
from aist_routines.msg  import (pickOrPlaceAction, pickOrPlaceGoal,
                                pickOrPlaceResult, pickOrPlaceFeedback)
from tf                 import transformations as tfs

######################################################################
#  class PickOrPlaceAction                                           #
######################################################################
class PickOrPlaceAction(object):
    def __init__(self, routines):
        super(PickOrPlaceAction, self).__init__()

        self._routines = routines
        self._server   = actionlib.SimpleActionServer("pickOrPlace",
                                                      pickOrPlaceAction,
                                                      self._execute_cb, False)
        self._server.start()
        self._client = actionlib.SimpleActionClient("pickOrPlace",
                                                    pickOrPlaceAction)
        self._client.wait_for_server()

    # Client stuffs
    def execute(self, robot_name, pose_stamped, pick, offset,
                approach_offset, departure_offset, speed_fast, speed_slow,
                wait=True, feedback_cb=None):
        goal = pickOrPlaceGoal()
        goal.robot_name       = robot_name
        goal.pose             = pose_stamped
        goal.pick             = pick
        goal.offset           = self._create_transform(offset)
        goal.approach_offset  = self._create_transform(approach_offset)
        goal.departure_offset = self._create_transform(departure_offset)
        goal.speed_fast       = speed_fast
        goal.speed_slow       = speed_slow
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
        return Transform(Vector3(*xyz), Quaternion(*q))

    def _execute_cb(self, goal):
        rospy.loginfo("*** Do %s ***", "picking" if goal.pick else "placing")
        routines = self._routines
        gripper  = routines.gripper(goal.robot_name)
        result   = pickOrPlaceResult()

        # Go to approach pose.
        rospy.loginfo("--- Go to approach pose. ---")
        if self._check_if_canceled(pickOrPlaceFeedback.MOVING):
            return
        success, _, _ = routines.go_to_pose_goal(
                             goal.robot_name,
                             routines.effector_target_pose(
                                 goal.pose,
                                 (goal.approach_offset.translation.x,
                                  goal.approach_offset.translation.y,
                                  goal.approach_offset.translation.z,
                                  goal.approach_offset.rotation.x,
                                  goal.approach_offset.rotation.y,
                                  goal.approach_offset.rotation.z,
                                  goal.approach_offset.rotation.w)),
                             goal.speed_fast if goal.pick else goal.speed_slow)
        if not success:
            result.result = pickOrPlaceResult.MOVE_FAILURE
            self._server.set_aborted(result, "Failed to go to approach pose")
            return

        # Approach pick/place pose.
        rospy.loginfo("--- Go to %s pose. ---",
                      "pick" if goal.pick else "place")
        if self._check_if_canceled(pickOrPlaceFeedback.APPROACHING):
            return
        if goal.pick:
            gripper.pregrasp(-1)               # Pregrasp (not wait)
        target_pose \
            = routines.effector_target_pose(goal.pose,
                                            (goal.offset.translation.x,
                                             goal.offset.translation.y,
                                             goal.offset.translation.z,
                                             goal.offset.rotation.x,
                                             goal.offset.rotation.y,
                                             goal.offset.rotation.z,
                                             goal.offset.rotation.w))
        routines.add_marker("pick_pose" if goal.pick else "place_pose",
                            target_pose)
        routines.publish_marker()
        success, _, _ = routines.go_to_pose_goal(goal.robot_name, target_pose,
                                                 goal.speed_slow)
        if not success:
            result.result = pickOrPlaceResult.APPROACH_FAILURE
            self._server.set_aborted(result, "Failed to approach target")
            return

        # Grasp/release at pick/place pose.
        if self._check_if_canceled(pickOrPlaceFeedback.GRASPING_OR_RELEASING):
            return
        if goal.pick:
            gripper.wait()                      # Wait for pregrasp completed
            gripper.grasp()
        else:
            gripper.release()

        # Go back to departure(pick) or approach(place) pose.
        rospy.loginfo("--- Go back to departure pose. ---")

        if self._check_if_canceled(pickOrPlaceFeedback.DEPARTING):
            return
        if goal.pick:
            gripper.postgrasp(-1)    # Postgrap (not wait)
            offset = goal.departure_offset
            speed  = goal.speed_slow
        else:
            offset = goal.approach_offset
            speed  = goal.speed_fast
        success, _, _ = routines.go_to_pose_goal(goal.robot_name,
                                                 routines.effector_target_pose(
                                                     goal.pose,
                                                     (offset.translation.x,
                                                      offset.translation.y,
                                                      offset.translation.z,
                                                      offset.rotation.x,
                                                      offset.rotation.y,
                                                      offset.rotation.z,
                                                      offset.rotation.w)),
                                                 speed)
        if not success:
            result.result = pickOrPlaceResult.DEPARTURE_FAILURE
            self._server.set_aborted(result, "Failed to depart from target")
            return
        if goal.pick:
            success = gripper.wait()  # Wait for postgrasp completed
            if success:
                rospy.loginfo("--- Pick succeeded. ---")
            else:
                rospy.logwarn("--- Pick failed. ---")

        if success:
            result.result = pickOrPlaceResult.SUCCESS
            self._server.set_succeeded(result, "Succeeded")
        else:
            result.result = pickOrPlaceResult.GRASP_FAILURE
            self._server.set_aborted(result, "Failed to grasp")

    def _check_if_canceled(self, state):
        self._server.publish_feedback(pickOrPlaceFeedback(state))
        if self._server.is_preempt_requested():
            self._server.set_preempted(pickOrPlaceResult(state))
            return True
        return False
