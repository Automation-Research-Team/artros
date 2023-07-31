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
import rospy, collections, numpy as np
from math                          import pi
from tf                            import transformations as tfs
from geometry_msgs.msg             import (PoseStamped, PointStamped,
                                           Vector3Stamped,
                                           Point, Quaternion, Vector3)
from aist_routines.KittingRoutines import KittingRoutines
from aist_routines.SweepAction     import Sweep
from aist_routines.msg             import SweepResult
from finger_pointing_msgs.msg      import (RequestHelpAction, RequestHelpGoal,
                                           RequestHelpResult,
                                           request_help, pointing)
from actionlib                     import SimpleActionClient
from visualization_msgs.msg        import Marker
from std_msgs.msg                  import ColorRGBA
from aist_utility.compat           import *

######################################################################
#  class HMIRoutines                                                 #
######################################################################
class HMIRoutines(KittingRoutines):
    """Implements HMI routines for aist robot system."""

    MarkerProps = collections.namedtuple('MarkerProps', 'id, scale, color')
    _marker_props = {
        'finger' : MarkerProps(0, (0.006, 0.014, 0.015), (0.0, 1.0, 1.0, 1.0)),
        'sweep'  : MarkerProps(1, (0.006, 0.014, 0.015), (1.0, 1.0, 0.0, 1.0))
        }

    def __init__(self, server='hmi_server'):
        super(HMIRoutines, self).__init__(self.request_help_and_sweep,
                                          self.cancel_request_help_and_sweep)

        self._ground_frame             = rospy.get_param('~ground_frame',
                                                         'ground')
        self._hmi_graspability_params  = rospy.get_param(
                                             '~hmi_graspability_parameters')
        self._graspability_params_back = None
        self._sweep_params             = rospy.get_param('~sweep_parameters')
        self._request_help_clnt        = SimpleActionClient(
                                             server + '/request_help',
                                             RequestHelpAction)
        self._sweep_clnt               = Sweep(self)
        self._marker_pub               = rospy.Publisher('pointing_marker',
                                                         Marker, queue_size=10)
        self._request_help_clnt.wait_for_server()

    # Interactive stuffs
    def print_help_messages(self):
        super(HMIRoutines, self).print_help_messages()
        print('=== HMI commands ===')
        print('  w: sWeep')
        print('  r: Request help')

    def interactive(self, key, robot_name, axis, speed):
        if key == 'w':
            bin_id = 'bin_' + raw_input('  bin id? ')
            self._clear_fail_poses()
            self.sweep_bin(bin_id)
            self.go_to_named_pose(self._current_robot_name, 'home')
        elif key == 'r':
            bin_id = 'bin_' + raw_input('  bin id? ')
            self.request_help_bin(bin_id)
        else:
            return super(HMIRoutines, self).interactive(key, robot_name, axis,
                                                        speed)
        return robot_name, axis, speed

    # Graspability stuffs
    def search_bin(self, bin_id, max_slant=pi/4):
        return super(HMIRoutines, self).search_bin(
                   bin_id,
                   0 if self.using_hmi_graspability_params else max_slant)

    @property
    def using_hmi_graspability_params(self):
        return not self._graspability_params_back is None

    def set_hmi_graspability_params(self, bin_id):
        bin_props = self._bin_props[bin_id]
        part_id   = bin_props['part_id']
        if self.using_hmi_graspability_params:
            rospy.logwarn('(hmi_demo) Already using graspability paramters for HMI demo.')
            return
        self._graspability_params_back = self._graspability_params[part_id]
        self._graspability_params[part_id] \
            = self._hmi_graspability_params[part_id]
        rospy.loginfo('(hmi_demo) Set graspability paramters for HMI demo.')

    def restore_original_graspability_params(self, bin_id):
        bin_props = self._bin_props[bin_id]
        part_id   = bin_props['part_id']
        if not self.using_hmi_graspability_params:
            rospy.logwarn('(hmi_demo) Already using original graspability paramters.')
            return
        self._graspability_params[part_id] = self._graspability_params_back
        self._graspability_params_back = None
        rospy.loginfo('(hmi_demo) Restore original graspability paramters.')

    # Sweep stuffs
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
        graspabilities = self.search_bin(bin_id, 0.0)

        # Attempt to sweep the item along y-axis.
        pose = PoseStamped(graspabilities.poses.header,
                           graspabilities.poses.poses[0])
        R    = tfs.quaternion_matrix((pose.pose.orientation.x,
                                      pose.pose.orientation.y,
                                      pose.pose.orientation.z,
                                      pose.pose.orientation.w))
        result = self._sweep(robot_name, pose, R[0:3, 1], part_id)
        return result == SweepResult.SUCCESS

    def _sweep(self, robot_name, target_pose, sweep_dir, part_id, wait=True):
        R = tfs.quaternion_matrix((target_pose.pose.orientation.x,
                                   target_pose.pose.orientation.y,
                                   target_pose.pose.orientation.z,
                                   target_pose.pose.orientation.w))
        xdir = np.cross(sweep_dir, R[0:3, 2])   # sweep_dir ^ surface_normal
        R[0:3, 0] = xdir/np.linalg.norm(xdir)
        R[0:3, 1] = sweep_dir/np.linalg.norm(sweep_dir)
        R[0:3, 2] = np.cross(R[0:3, 0], R[0:3, 1])
        target_pose.pose.orientation = Quaternion(
                                           *tfs.quaternion_from_matrix(R))
        params = self._sweep_params[part_id]
        return self._sweep_clnt.send_goal(robot_name, target_pose,
                                          params['sweep_length'],
                                          params['sweep_offset'],
                                          params['approach_offset'],
                                          params['departure_offset'],
                                          params['speed_fast'],
                                          params['speed_slow'],
                                          wait)

    # Request help stuffs
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
        message    = '[Request_testing]_Please_specify_sweep_direction.'

        # Search for graspabilities.
        graspabilities = self.search_bin(bin_id)
        pose           = PoseStamped(graspabilities.poses.header,
                                     graspabilities.poses.poses[0])

        # Send request and receive response.
        response = self._request_help(robot_name, pose, part_id, message)
        if response.pointing_state == pointing.SWEEP_RES:
            self._publish_marker('finger', response.header,
                                 response.finger_pos, response.finger_dir)
            sweep_dir = self._compute_sweep_dir(pose, response)
            self._publish_marker('sweep', pose.header, pose.pose.position,
                                 Vector3(*sweep_dir))
        print('*** response=%s' % str(response))

    def request_help_and_sweep(self, robot_name, pose, part_id):
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
        @return:           False if picking task should be aborted
        """
        message = 'Picking failed! Please sepcify sweep direction.'
        while True:
            response = self._request_help(robot_name, pose, part_id, message)

            if response.pointing_state == pointing.SWEEP_RES:
                rospy.loginfo('(hmi_demo) Sweep direction given.')
                self._publish_marker('finger', response.header,
                                     response.finger_pos, response.finger_dir)
                sweep_dir = self._compute_sweep_dir(pose, response)
                self._publish_marker('sweep', pose.header, pose.pose.position,
                                     Vector3(*sweep_dir))

                result = self._sweep(robot_name, pose, sweep_dir, part_id)

                if result == SweepResult.SUCCESS:
                    break
                elif result == SweepResult.DEPARTURE_FAILURE:
                    return False
                elif result == SweepResult.PREEMPTED:
                    rospy.logwarn('(hmi_demo) Preempted while sweeping!')
                    break
                message = 'Planning for sweep failed! Please specify another sweep direction'
            elif response.pointing_state == pointing.RECAPTURE_RES:
                rospy.loginfo('(hmi_demo) Recapture required.')
                break
            else:
                rospy.logwarn('(hmi_demo) Preempted while requesting help!')
                break
        return True

    def cancel_request_help_and_sweep(self):
        self._request_help_clnt.cancel_goal()
        self._sweep_clnt.cancel_goal()

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
        self._request_help_clnt.send_goal(
            RequestHelpGoal(req), feedback_cb=self._request_help_feedback_cb)
        self._request_help_clnt.wait_for_result()
        return self._request_help_clnt.get_result().response

    def _request_help_feedback_cb(self, feedback):
        response = feedback.response
        self._publish_marker('finger', response.header,
                             response.finger_pos, response.finger_dir)

    def _compute_sweep_dir(self, pose, response):
        fpos = self.listener.transformPoint(
                   pose.header.frame_id,
                   PointStamped(response.header, response.finger_pos)).point
        fdir = self.listener.transformVector3(pose.header.frame_id,
                                              Vector3Stamped(
                                                  response.header,
                                                  response.finger_dir)).vector
        ppos = pose.pose.position
        fnrm = np.cross((fdir.x, fdir.y, fdir.z),
                        (ppos.x - fpos.x, ppos.y - fpos.y, ppos.z - fpos.z))
        gnrm = self.listener.transformVector3(pose.header.frame_id,
                                              Vector3Stamped(
                                                  response.header,
                                                  Vector3(0, 0, 1))).vector
        sdir = np.cross(fnrm, (gnrm.x, gnrm.y, gnrm.z))
        return tuple(sdir / np.linalg.norm(sdir))

    # Marker stuffs
    def _delete_markers(self):
        """
        Delete all markers published by _marker_pub.
        """
        marker        = Marker()
        marker.action = Marker.DELETEALL
        marker.ns     = 'pointing'
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
        marker.ns           = 'pointing'
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
