#!/usr/bin/env python
#
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
import rospy, collections, copy
import numpy as np
from math                     import pi, radians, degrees
from geometry_msgs.msg        import (QuaternionStamped, PoseStamped,
                                      PointStamped, Vector3Stamped,
                                      Point, Quaternion, Vector3)
from aist_routines            import AISTBaseRoutines
from aist_routines.msg        import PickOrPlaceResult, SweepResult
from finger_pointing_msgs.msg import (RequestHelpAction, RequestHelpGoal,
                                      RequestHelpResult,
                                      request_help, pointing)
from actionlib                import SimpleActionClient
from actionlib_msgs.msg       import GoalStatus
from tf                       import transformations as tfs
from visualization_msgs.msg   import Marker
from std_msgs.msg             import ColorRGBA

######################################################################
#  class HMIRoutines                                                 #
######################################################################
class HMIRoutines(AISTBaseRoutines):
    """Implements HMI routines for aist robot system."""

    MarkerProps = collections.namedtuple('MarkerProps',
                                         'id, scale, color')
    _marker_props = {
        'finger' : MarkerProps(0, (0.004, 0.010, 0.015), (0.0, 1.0, 1.0, 0.8)),
        'sweep'  : MarkerProps(1, (0.004, 0.010, 0.015), (1.0, 1.0, 0.0, 0.8))
        }

    def __init__(self, server='hmi_server'):
        super(HMIRoutines, self).__init__()

        self._ground_frame       = rospy.get_param('~ground_frame', 'ground')
        self._bin_props          = rospy.get_param('~bin_props')
        self._part_props         = rospy.get_param('~part_props')
        self._hmi_graspability_params \
            = rospy.get_param('~hmi_graspability_parameters')
        self._graspability_params_back = None
        self._current_robot_name = None
        self._fail_poses         = []
        self._request_help_clnt  = SimpleActionClient(server + '/request_help',
                                                      RequestHelpAction)
        self._marker_pub         = rospy.Publisher("pointing_marker",
                                                   Marker, queue_size=10)
        self._request_help_clnt.wait_for_server()

    @property
    def nbins(self):
        return len(self._bin_props)

    @property
    def current_robot_name(self):
        return self._current_robot_name

    @property
    def using_hmi_graspability_params(self):
        return not self._graspability_params_back is None

    ###----- main procedure
    def demo(self, bin_id, max_attempts=1):
        while kitting.attempt_bin(bin_id, 5):
            pass
        self.go_to_named_pose(self.current_robot_name, 'home')

    def search(self, bin_id, max_slant=pi/4):
        """
        Search graspability points from the specified bin.

        @type  bin_id:    str
        @param bin_id:    ID specifying the bin
        @type  max_slant: float
        @param max_slant: maximum angle between normal of the graspability
                          point and the gravity direction
        @return:          True if succeeded
        """
        bin_props  = self._bin_props[bin_id]
        part_props = self._part_props[bin_props['part_id']]
        self.graspability_send_goal(part_props['robot_name'],
                                    bin_props['part_id'], bin_props['mask_id'])
        self.camera(part_props['camera_name']).trigger_frame()

        orientation = QuaternionStamped()
        orientation.header.frame_id = self.reference_frame
        orientation.quaternion = Quaternion(0, 0, 0, 1)
        return self.graspability_wait_for_result(orientation, max_slant)

    def sweep_bin(self, bin_id):
        """
        Search graspability points from the specified bin and sweep the one
        with the highest score.

        @type  bin_id: str
        @param bin_id: ID specifying the bin
        @return:       True if sweep succeeded
        """
        bin_props  = self._bin_props[bin_id]
        part_id    = bin_props['part_id']
        part_props = self._part_props[part_id]
        robot_name = part_props['robot_name']

        # If using a different robot from the former, move it back to home.
        if self._current_robot_name is not None and \
           self._current_robot_name != robot_name:
            self.go_to_named_pose(self._current_robot_name, 'back')
        self._current_robot_name = robot_name

        # Move to 0.15m above the bin if the camera is mounted on the robot.
        if self._is_eye_on_hand(robot_name, part_props['camera_name']):
            self.go_to_frame(robot_name, bin_props['name'], (0, 0, 0.15))

        # Search for graspabilities.
        poses, _ = self.search(bin_id, 0.0)

        # Attempt to sweep the item along y-axis.
        pose = PoseStamped(poses.header, poses.poses[0])
        R    = tfs.quaternion_matrix((pose.pose.orientation.x,
                                      pose.pose.orientation.y,
                                      pose.pose.orientation.z,
                                      pose.pose.orientation.w))
        result = self.sweep(robot_name, pose, R[0:3, 1], part_id)
        return result == SweepResult.SUCCESS

    def attempt_bin(self, bin_id, max_attempts=5):
        bin_props  = self._bin_props[bin_id]
        part_id    = bin_props['part_id']
        part_props = self._part_props[part_id]
        robot_name = part_props['robot_name']

        # If using a different robot from the former, move it back to home.
        if self._current_robot_name is not None and \
           self._current_robot_name != robot_name:
            self.go_to_named_pose(self._current_robot_name, 'back')
        self._current_robot_name = robot_name

        # Move to 0.15m above the bin if the camera is mounted on the robot.
        if self._is_eye_on_hand(robot_name, part_props['camera_name']):
            self.go_to_frame(robot_name, bin_props['name'], (0, 0, 0.15))

        # Search for graspabilities.
        poses, _ = self.search(bin_id)

        # Attempt to pick the item.
        for p in poses.poses:
            pose = PoseStamped(poses.header, p)
            if self._is_close_to_fail_poses(pose):
                continue

            result = self.pick(robot_name, pose, part_id)       # Pick!

            if result == PickOrPlaceResult.SUCCESS:
                result = self.place_at_frame(robot_name,
                                             part_props['destination'],
                                             part_id)
                return result == PickOrPlaceResult.SUCCESS
            elif result == PickOrPlaceResult.MOVE_FAILURE or \
                 result == PickOrPlaceResult.APPROACH_FAILURE:
                self._fail_poses.append(pose)
            elif result == PickOrPlaceResult.DEPARTURE_FAILURE:
                raise RuntimeError('Failed to depart from pick/place pose')
            elif result == PickOrPlaceResult.GRASP_FAILURE:
                # Request help to the VR side and sweep
                rospy.logwarn('(hmi_demo) Pick failed. Request help!')
                message = 'Picking failed! Please specify sweep direction.'
                while self._request_help_and_sweep(robot_name, pose, part_id,
                                                   message):
                    message = 'Planning for sweeping failed! Please specify another sweep direction.'
                self._restore_original_graspability_params(bin_id)
                return True

        if self.using_hmi_graspability_params:
            return False
        else:
            self._set_hmi_graspability_params(bin_id)
            return True

    def request_help_bin(self, bin_id):
        """
        Search graspability points from the specified bin and request
        finger direction for computing direction to sweep the one with the
        highest score. Computed sweep direction is then visualized.

        @type  bin_id: str
        @param bin_id: ID specifying the bin
        """
        bin_props  = self._bin_props[bin_id]
        part_id    = bin_props['part_id']
        part_props = self._part_props[part_id]
        robot_name = part_props['robot_name']
        message    = '[Request testing] Please specify sweep direction.'

        # Search for graspabilities.
        poses, _ = self.search(bin_id)
        pose     = PoseStamped(poses.header, poses.poses[0])

        # pose = PoseStamped()
        # pose.header.frame_id  = 'workspace_center'
        # pose.header.stamp     = rospy.Time.now()
        # pose.pose.position    = Point(0, 0, 0.1)
        # pose.pose.orientation = Quaternion(0, 0, 0, 1)

        # Send request and receive response.
        res = self._request_help(robot_name, pose, part_id, message)
        if res.pointing_state == pointing.SWEEP_RES:
            self._publish_marker('finger',
                                 res.header, res.finger_pos, res.finger_dir)
            sweep_dir = self._compute_sweep_dir(pose, res)
            self._publish_marker('sweep', pose.header, pose.pose.position,
                                 Vector3(*sweep_dir))
        print('*** response=%s' % str(res))

    # Request help stuffs
    def _request_help(self, robot_name, pose, part_id, message):
        """
        Request finger direction for the specified graspability point
        and receive response.

        @type  robot_name: str
        @param robot_name: name of the robot
        @type  pose:       geometry_msgs.msg.PoseStamped
        @param pose:       pose of the graspability point to be sweeped
        @type  part_id:    str
        @param part_id:    ID for specifying part
        @type  message:    str
        @param message:    message to be displayed to the operator of VR side
        @return:           response with finger direction from VR side
        """
        req = request_help()
        req.robot_name = robot_name
        req.item_id    = part_id
        req.pose       = self.listener.transformPose(self._ground_frame, pose)
        req.request    = request_help.SWEEP_DIR_REQ
        req.message    = message
        self._request_help_clnt.send_goal(RequestHelpGoal(req),
                                         feedback_cb=self._feedback_cb)
        self._request_help_clnt.wait_for_result()
        return self._request_help_clnt.get_result().response

    def _request_help_and_sweep(self, robot_name, pose, part_id, message):
        """
        Request finger direction for the specified graspability point
        and perform sweeping the point in the direction computed from
        the received response.

        @type  robot_name: str
        @param robot_name: name of the robot
        @type  pose:       geometry_msgs.msg.PoseStamped
        @param pose:       pose of the graspability point to be sweeped
        @type  part_id:    str
        @param part_id:    ID for specifying part
        @type  message:    str
        @param message:    message to be displayed to the operator of VR side
        @return:           True if needed to send request again
        """
        res = self._request_help(robot_name, pose, part_id, message)

        if res.pointing_state == pointing.SWEEP_RES:
            rospy.loginfo('(hmi_demo) Sweep direction given.')
            self._publish_marker('finger',
                                 res.header, res.finger_pos, res.finger_dir)
            sweep_dir = self._compute_sweep_dir(pose, res)  # Compute direction
            self._publish_marker('sweep', pose.header, pose.pose.position,
                                 Vector3(*sweep_dir))
            result = self.sweep(robot_name, pose, sweep_dir, part_id)
            if result == SweepResult.MOVE_FAILURE or \
               result == SweepResult.APPROACH_FAILURE:
                return True                     # Need to send request again
            elif result == SweepResult.DEPARTURE_FAILURE:
                raise RuntimeError('Failed to depart from sweep pose')

        elif res.pointing_state == pointing.RECAPTURE_RES:
            rospy.loginfo('(hmi_demo) Recapture required.')
        else:
            rospy.logerr('(hmi_demo) Unknown command received!')
        return False                            # No more requests required.

    def _compute_sweep_dir(self, pose, res):
        fpos = self.listener.transformPoint(pose.header.frame_id,
                                            PointStamped(res.header,
                                                         res.finger_pos)).point
        fdir = self.listener.transformVector3(pose.header.frame_id,
                                              Vector3Stamped(
                                                  res.header,
                                                  res.finger_dir)).vector
        ppos = pose.pose.position
        fnrm = np.cross((fdir.x, fdir.y, fdir.z),
                        (ppos.x - fpos.x, ppos.y - fpos.y, ppos.z - fpos.z))
        gnrm = self.listener.transformVector3(pose.header.frame_id,
                                              Vector3Stamped(
                                                  res.header,
                                                  Vector3(0, 0, 1))).vector
        sdir = np.cross(fnrm, (gnrm.x, gnrm.y, gnrm.z))
        return tuple(sdir / np.linalg.norm(sdir))

    def _feedback_cb(self, feedback):
        res = feedback.response
        self._publish_marker('finger',
                             res.header, res.finger_pos, res.finger_dir)

    # Marker stuffs
    def _delete_markers(self):
        """
        Delete all markers published by _marker_pub.
        """
        marker        = Marker()
        marker.action = Marker.DELETEALL
        marker.ns     = "pointing"
        self._marker_pub.publish(marker)

    def _publish_marker(self, marker_type, header, pos, dir, lifetime=15):
        """
        Publish arrow marker with specified start point and direction.

        @type  pos: geometry_msgs.msg.Point
        @param pos: start point of the arrow marker
        @type  dir: geometry_msgs.msg.Vector3
        @param dir: direction of the arrow marker
        """
        marker_prop = HMIRoutines._marker_props[marker_type]

        if marker_type == 'finger':
            workspace_center = PointStamped()
            workspace_center.header.stamp    = header.stamp
            workspace_center.header.frame_id = 'workspace_center'
            workspace_center.point           = Point(0, 0, 0)
            t = (self.listener.transformPoint(header.frame_id,
                                              workspace_center).point.z
                 - pos.z) / dir.z
        else:
            t = 0.03

        marker              = Marker()
        marker.header       = header
        marker.header.stamp = rospy.Time.now()
        marker.ns           = "pointing"
        marker.id           = marker_prop.id
        marker.type         = Marker.ARROW
        marker.action       = Marker.ADD
        marker.scale        = Vector3(*marker_prop.scale)
        marker.color        = ColorRGBA(*marker_prop.color)
        marker.lifetime     = rospy.Duration(lifetime)
        marker.points.append(pos)
        marker.points.append(Point(pos.x + t*dir.x,
                                   pos.y + t*dir.y,
                                   pos.z + t*dir.z))
        self._marker_pub.publish(marker)

    # Misc stuffs
    def _set_hmi_graspability_params(self, bin_id):
        if self.using_hmi_graspability_params:
            rospy.logwarn('(hmi_demo) Already using graspability paramters for HMI demo.')
            return
        part_id = self._bin_props[bin_id]['part_id']
        self._graspability_params_back = self._graspability_params[part_id]
        self._graspability_params[part_id] \
            = self._hmi_graspability_params[part_id]
        rospy.loginfo('(hmi_demo) Set graspability paramters for HMI demo.')

    def _restore_original_graspability_params(self, bin_id):
        if not self.using_hmi_graspability_params:
            rospy.logwarn('(hmi_demo) Already using original graspability paramters.')
            return
        part_id = self._bin_props[bin_id]['part_id']
        self._graspability_params[part_id] = self._graspability_params_back
        self._graspability_params_back = None
        rospy.loginfo('(hmi_demo) Restore original graspability paramters.')

    def clear_fail_poses(self):
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


if __name__ == '__main__':

    rospy.init_node('hmi_demo', anonymous=True)

    with HMIRoutines() as hmi:
        while not rospy.is_shutdown():
            print('============ hmi_ procedures ============ ')
            print('  b: Create a backgroud image')
            print('  m: Create a mask image')
            print('  s: Search graspabilities')
            print('  a: Attempt to pick and place')
            print('  A: Repeat attempts to pick and place')
            print('  w: attempts to sWeep')
            print('  h: request help')
            print('  g: Grasp')
            print('  r: Release')
            print('  H: Move all robots to Home')
            print('  B: Move all robots to Back')
            print('  q: Quit')

            try:
                key = raw_input('>> ')
                if key == 'q':
                    if hmi.current_robot_name is not None:
                        hmi.go_to_named_pose(hmi.current_robot_name, 'home')
                    break
                elif key == 'H':
                    hmi.go_to_named_pose('all_bots', 'home')
                elif key == 'B':
                    hmi.go_to_named_pose('all_bots', 'back')
                elif key == 'b':
                    hmi.create_background_image('a_phoxi_m_camera')
                elif key == 'm':
                    hmi.create_mask_image('a_phoxi_m_camera', hmi.nbins)
                elif key == 's':
                    bin_id = 'bin_' + raw_input('  bin id? ')
                    hmi.search(bin_id)
                elif key == 'a':
                    bin_id = 'bin_' + raw_input('  bin id? ')
                    hmi.clear_fail_poses()
                    hmi.attempt_bin(bin_id, 5)
                    hmi.go_to_named_pose(hmi.current_robot_name, 'home')
                elif key == 'A':
                    bin_id = 'bin_' + raw_input('  bin id? ')
                    hmi.clear_fail_poses()
                    while hmi.attempt_bin(bin_id, 5):
                        pass
                    hmi.go_to_named_pose(hmi.current_robot_name, 'home')
                elif key == 'w':
                    bin_id = 'bin_' + raw_input('  bin id? ')
                    hmi.clear_fail_poses()
                    hmi.sweep_bin(bin_id)
                    hmi.go_to_named_pose(hmi.current_robot_name, 'home')
                elif key == 'h':
                    bin_id = 'bin_' + raw_input('  bin id? ')
                    hmi.request_help_bin(bin_id)
                elif key == 'c':
                    self.pick_or_place_cancel()
                elif key == 'g':
                    if hmi.current_robot_name is not None:
                        hmi.grasp(hmi.current_robot_name)
                elif key == 'r':
                    if hmi.current_robot_name is not None:
                        hmi.release(hmi.current_robot_name)
            except Exception as e:
                print('(hmi_demo) ' + e.message)
