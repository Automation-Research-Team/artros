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
import rospy, threading
import numpy as np
from actionlib          import SimpleActionServer, SimpleActionClient
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg  import Transform, Vector3, Quaternion
from aist_routines.msg  import (AttemptBinAction, AttemptBinGoal,
                                AttemptBinResult, AttemptBinFeedback)
from tf                 import transformations as tfs

######################################################################
#  class AttemptBin                                                  #
######################################################################
class AttemptBin(SimpleActionClient):
    def __init__(self, routines):
        SimpleActionClient.__init__(self, "attempt_bin", AttemptBinAction)

        self._routines  = routines
        self._condition = threading.Condition()
        self._stage     = None
        self._server    = SimpleActionServer("attempt_bin", AttemptBinAction,
                                             self._execute_cb, False)
        self._server.start()
        self.wait_for_server()

    # Client stuffs
    def send_goal(self, bin_id, once, done_cb=None, active_cb=None):
        SimpleActionClient.send_goal(self, AttemptBin(bin_id, once),
                                     self._done_cb)

    def _done_cb(self, status, result):
        self.go_to_named_pose(self._current_robot_name, 'home')

    # Server stuffs
    def shutdown(self):
        self._server.__del__()

    def _execute_cb(self, goal):
        remained     = True
        poses        = None
        place_offset = 0.020
        self._clear_fail_poses()
        while remained:
            remained, poses = self._attempt_bin(goal.bin_id, poses,
                                                place_offset)
            if not self._server.is_active():
                return
            if goal.once:
                break
            place_offset = -place_offset
        self._server.set_succeeded()
        rospy.loginfo('(AttemptBin) SUCCEEDED')

    def _attempt_bin(self, bin_id, poses, place_offset)
        bin_props  = self._bin_props[goal.bin_id]
        part_id    = bin_props['part_id']
        part_props = self._part_props[part_id]
        robot_name = part_props['robot_name']
        routines   = self._routines

        # If using a different robot from the former, move it back to home.
        if self._current_robot_name is not None and \
           self._current_robot_name != robot_name:
            routines.go_to_named_pose(self._current_robot_name, 'back')
        self._current_robot_name = robot_name

        # Move to 0.15m above the bin if the camera is mounted on the robot.
        if self._is_eye_on_hand(robot_name, part_props['camera_name']):
            routines.go_to_frame(robot_name, bin_props['name'], (0, 0, 0.15))

        # Search for graspabilities.
        if poses is None:
            poses = routines.search_bin(bin_id).poses

        if not self._server.is_active():
            return False, None

        # Attempt to pick the item.
        nattempts = 0
        for p in poses.poses:
            if nattempts == max_attempts:
                break

            pose = PoseStamped(poses.header, p)
            if self._is_close_to_fail_poses(pose):
                continue

            result = routines.pick(robot_name, pose, part_id)
            if not self._server.is_active():
                break

            print('*** pick_result=%d' % result)

            if result == PickOrPlaceResult.SUCCESS:
                routines.place_at_frame(robot_name, part_props['destination'],
                                        part_id,
                                        offset=(0.0, place_offset, 0.0),
                                        wait=False)
                routines.pick_or_place_wait_for_stage(
                    PickOrPlaceFeedback.APPROACHING)
                poses  = routines.search_bin(bin_id).poses
                result = routines.pick_or_place_wait_for_result()
                if not self._server.is_active():
                    break
                return result == PickOrPlaceResult.SUCCESS, poses
            elif result == PickOrPlaceResult.MOVE_FAILURE or \
                 result == PickOrPlaceResult.APPROACH_FAILURE:
                self._fail_poses.append(pose)
            elif result == PickOrPlaceResult.DEPARTURE_FAILURE:
                self._server.set_aborted()
                rospy.logerr('(AttemptBin) Failed to depart from pick/place pose')
                break
            elif result == PickOrPlaceResult.GRASP_FAILURE:
                self._fail_poses.append(pose)
                nattempts += 1

        return False, None

    def _preempt_cb(self):
        self._routines.pick_or_place_cancel_goal()
        self._server.set_preempted()
        rospy.logwarn('(AttemptBin) CANCELLED')
