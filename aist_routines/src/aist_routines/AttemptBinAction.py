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
from math                  import pi, radians, degrees, cos, sin, sqrt
from geometry_msgs.msg     import (PoseStamped, QuaternionStamped,
                                   Transform, Vector3, Quaternion)
from actionlib             import SimpleActionServer, SimpleActionClient
from actionlib_msgs.msg    import GoalStatus
from aist_routines.msg     import (PickOrPlaceResult, PickOrPlaceFeedback,
                                   SweepResult,
                                   AttemptBinAction, AttemptBinGoal,
                                   AttemptBinResult, AttemptBinFeedback)
from aist_graspability.msg import Graspabilities
from tf                    import transformations as tfs

######################################################################
#  class AttemptBin                                                  #
######################################################################
class AttemptBin(SimpleActionClient):
    def __init__(self, routines):
        SimpleActionClient.__init__(self, "attempt_bin", AttemptBinAction)

        self._routines           = routines
        self._bin_props          = rospy.get_param('~bin_props')
        self._part_props         = rospy.get_param('~part_props')
        self._current_robot_name = None
        self._fail_poses         = []
        self._condition          = threading.Condition()
        self._stage              = None
        self._server             = SimpleActionServer("attempt_bin",
                                                      AttemptBinAction,
                                                      self._execute_cb, False)
        self._server.register_preempt_callback(self._preempt_cb)
        self._server.start()
        self.wait_for_server()

    @property
    def current_robot_name(self):
        return self._current_robot_name

    # Graspability stuffs
    def create_mask_image(self, camera_name):
        self._routines.create_mask_image(camera_name, len(self._bin_props))

    def search_bin(self, bin_id, max_slant=pi/4):
        bin_props  = self._bin_props[bin_id]
        part_id    = bin_props['part_id']
        part_props = self._part_props[part_id]
        routines   = self._routines
        routines.graspability_send_goal(part_props['robot_name'],
                                        part_id, bin_props['mask_id'])
        routines.camera(part_props['camera_name']).trigger_frame()
        return routines.graspability_wait_for_result(
                   bin_props['name'],
                   lambda pose, max_slant=max_slant:
                       self._pose_filter(pose, max_slant))

    # Client stuffs
    def send_goal(self, bin_id, once, max_attempts,
                  done_cb=None, active_cb=None):
        SimpleActionClient.send_goal(self,
                                     AttemptBinGoal(bin_id, once, max_attempts),
                                     done_cb, active_cb)

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
                                                place_offset, goal.max_attempts)
            if not self._server.is_active():
                return
            if goal.once:
                break
            place_offset = -place_offset
        self._server.set_succeeded()
        rospy.loginfo('(AttemptBin) SUCCEEDED')

    def _attempt_bin(self, bin_id, poses, place_offset, max_attempts):
        bin_props  = self._bin_props[bin_id]
        part_id    = bin_props['part_id']
        part_props = self._part_props[part_id]
        robot_name = part_props['robot_name']
        routines   = self._routines

        # If using a different robot from the former, move it back to home.
        if self.current_robot_name is not None and \
           self.current_robot_name != robot_name:
            routines.go_to_named_pose(self.current_robot_name, 'back')
        self._current_robot_name = robot_name

        # Move to 0.15m above the bin if the camera is mounted on the robot.
        if self._is_eye_on_hand(robot_name, part_props['camera_name']):
            routines.go_to_frame(robot_name, bin_props['name'], (0, 0, 0.15))

        # Search for graspabilities.
        if poses is None:
            poses = self.search_bin(bin_id).poses

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

            if result == PickOrPlaceResult.SUCCESS:
                routines.place_at_frame(robot_name, part_props['destination'],
                                        part_id,
                                        offset=(0.0, place_offset, 0.0),
                                        wait=False)
                routines.pick_or_place_wait_for_stage(
                    PickOrPlaceFeedback.APPROACHING)
                poses  = self.search_bin(bin_id).poses
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

    # Utilities
    def _clear_fail_poses(self):
        self._fail_poses = []

    def _is_eye_on_hand(self, robot_name, camera_name):
        return camera_name == robot_name + '_camera'

    def _is_close_to_fail_poses(self, pose):
        for fail_pose in self._fail_poses:
            if self._is_close_to_fail_pose(pose, fail_pose, 0.005):
                return True
        return False

    def _is_close_to_fail_pose(self, pose, fail_pose, tolerance):
        position      = pose.pose.position
        fail_position = fail_pose.pose.position
        if abs(position.x - fail_position.x) > tolerance or \
           abs(position.y - fail_position.y) > tolerance or \
           abs(position.z - fail_position.z) > tolerance:
            return False
        return True

    def _pose_filter(self, pose, max_slant):
        if pose.position.z < 0.002:
            return None

        T = tfs.quaternion_matrix((pose.orientation.x, pose.orientation.y,
                                   pose.orientation.z, pose.orientation.w))
        normal = T[0:3, 2]      # local Z-axis at the graspability point
        up     = np.array((0, 0, 1))
        a = np.dot(normal, up)
        b = cos(max_slant)
        if a < b:
            p = sqrt((1.0 - b*b)/(1.0 - a*a))
            q = b - a*p
            R = np.identity(4, dtype=np.float32)
            R[0:3, 2] = p*normal + q*up                   # fixed Z-axis
            R[0:3, 1] = self._normalize(np.cross(R[0:3, 2], T[0:3, 0]))
            R[0:3, 0] = np.cross(R[0:3, 1], R[0:3, 2])
            pose.orientation = Quaternion(*tfs.quaternion_from_matrix(R))
        return pose

    def _normalize(self, x):
        return x / sqrt(np.dot(x, x))
