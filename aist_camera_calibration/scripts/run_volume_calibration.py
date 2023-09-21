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
import rospy, actionlib, rospkg, copy, yaml
from std_srvs.srv                import Empty, Trigger
from geometry_msgs.msg           import PoseStamped, Pose, Point, Quaternion
from actionlib_msgs.msg          import GoalStatus
from tf                          import transformations as tfs
from aist_routines               import AISTBaseRoutines
from aist_camera_calibration.srv import GetSampleList, ComputeCalibration
from aist_camera_calibration.msg import TakeSampleAction, TakeSampleGoal
from aist_utility.compat         import *

######################################################################
#  class CameraCalibrationRoutines                                  #
######################################################################
class CameraCalibrationRoutines(AISTBaseRoutines):
    def __init__(self):
        super(CameraCalibrationRoutines, self).__init__()

        self._camera_name = rospy.get_param('~camera_name', 'live_camera')
        self._robot_name  = rospy.get_param('~robot_name', 'b_bot')
        self._initpose    = rospy.get_param('~initpose', [])
        self._keyposes    = rospy.get_param('~keyposes', [])
        self._speed       = rospy.get_param('~speed', 1)
        self._sleep_time  = rospy.get_param('~sleep_time', 0.5)

        ns = '/camera_calibrator'
        self._get_sample_list     = rospy.ServiceProxy(ns + '/get_sample_list',
                                                       GetSampleList)
        self._compute_calibration = rospy.ServiceProxy(
                                        ns + '/compute_calibration',
                                            ComputeCalibration)
        self._save_calibration    = rospy.ServiceProxy(
                                        ns + '/save_calibration', Trigger)
        self._reset               = rospy.ServiceProxy(ns + '/reset', Empty)
        self._take_sample         = actionlib.SimpleActionClient(
                                        ns + '/take_sample', TakeSampleAction)

    def run(self):
        # Reset pose
        self.go_to_named_pose(self._robot_name, "home")
        self.print_help_messages()
        print('')

        axis = 'Y'

        while not rospy.is_shutdown():
            prompt = '{:>5}:{}>> '.format(axis, self.format_pose(
                                                    self.get_current_pose(
                                                        self._robot_name)))
            key = raw_input(prompt)
            _, axis, _ = self.interactive(key, self._robot_name, axis,
                                          self._speed)

    # interactive stuffs
    def print_help_messages(self):
        super(CameraCalibrationRoutines, self).print_help_messages()
        print('=== Calibration commands ===')
        print('  init:  go to initial pose')
        print('  calib: do calibration')

    def interactive(self, key, robot_name, axis, speed):
        if key == 'init':
            self.go_to_initpose()
        elif key == 'calib':
            self.calibrate()
            self.go_to_named_pose(self._robot_name, 'home')
        else:
            return super(CameraCalibrationRoutines, self) \
                  .interactive(key, robot_name, axis, speed)
        return robot_name, axis, speed

    def go_to_initpose(self):
        self._move(self._initpose, self._robot_effector_frame)

    def calibrate(self):
        self._reset()

        # Reset pose
        self.go_to_named_pose(self._robot_name, 'home')
        self.go_to_initpose()

        # Collect samples over pre-defined poses
        keyposes = self._keyposes
        for i, keypose in enumerate(keyposes, 1):
            print('\n*** Keypose [%d/%d]: Try! ***' % (i, len(keyposes)))
            self._move_to(keypose, i, 1)
            print('*** Keypose [%d/%d]: Completed. ***' % (i, len(keyposes)))

        res = self._compute_calibration()
        if not res.success:
            rospy.logerr(res.message)
            return
        rospy.loginfo(res.message)

        for camera_name, intrinsic, pose in zip(res.camera_names,
                                                res.intrinsics, res.poses):
            self._save_camera_pose(camera_name, intrinsic, pose)

        res = self._save_calibration()
        if res.success:
            rospy.loginfo(res.message)
        else:
            rospy.logerr(res.message)

    # Move stuffs
    def _move_to(self, subpose, keypose_num, subpose_num):
        if not self._move(subpose, self._robot_effector_frame):
            return False

        rospy.sleep(self._sleep_time)  # Wait for the robot to settle.
        self._take_sample.send_goal(TakeSampleGoal())
        self.trigger_frame(self._camera_name)
        if not self._take_sample.wait_for_result(rospy.Duration(5.0)):
            self._take_sample.cancel_goal()  # timeout expired
            rospy.logerr('TakeSampleAction: timeout expired')
            return False
        if self._take_sample.get_state() != GoalStatus.SUCCEEDED:
            rospy.logerr('TakeSampleAction: not in succeeded state')
            return False

        result = self._take_sample.get_result()
        for correspondences in result.correspondences_set:
            print('[%s] %d point correspondences w.r.t. %s'
                  % (correspondences.camera_name,
                     len(correspondences.correspondences),
                     correspondences.reference_frame))

        return True

    def _move(self, xyzrpy, end_effector_link):
        pose = self.pose_from_xyzrpy(xyzrpy)
        print('  move to ' + self.format_pose(pose))
        success, _, current_pose \
            = self.go_to_pose_goal(self._robot_name, pose, self._speed,
                                   end_effector_link=end_effector_link,
                                   move_lin=True)
        print('  reached ' + self.format_pose(current_pose))
        return success

    def _save_camera_pose(self, camera_name, intrinsic, pose):
        # Frame to which the camera attached
        camera_parent_frame = rospy.get_param('~camera_parent_frame', '')
        if camera_parent_frame == '':
            return

        # Get camera base frame whose parent is camera_parent_frame.
        camera_frame      = intrinsic.header.frame_id
        stamp             = intrinsic.header.stamp
        reference_frame   = pose.header.frame_id
        chain             = self.listener.chain(camera_parent_frame, stamp,
                                                camera_frame, stamp,
                                                camera_parent_frame)
        camera_base_frame = chain[-2]

        # Compute transform from camera base frame to its parent.
        rTc = self.listener.fromTranslationRotation((pose.pose.position.x,
                                                     pose.pose.position.y,
                                                     pose.pose.position.z),
                                                    (pose.pose.orientation.x,
                                                     pose.pose.orientation.y,
                                                     pose.pose.orientation.z,
                                                     pose.pose.orientation.w))
        pTr = self.listener.fromTranslationRotation(
                                *self.listener.lookupTransform(
                                    camera_parent_frame,
                                    reference_frame, stamp))
        cTb = self.listener.fromTranslationRotation(
                                *self.listener.lookupTransform(
                                    camera_frame, camera_base_frame, stamp))
        pTb = tfs.concatenate_matrices(pTr, rTc, cTb)

        # Convert the transform to xyz-rpy representation.
        xyz  = list(map(float, tfs.translation_from_matrix(pTb)))
        rpy  = list(map(float, tfs.euler_from_matrix(pTb)))
        data = {'parent': camera_parent_frame,
                'child' : camera_base_frame,
                'origin': xyz + rpy}
        print(data)
        # Save the transform.
        filename = rospkg.RosPack().get_path('aist_camera_calibration') \
                 + '/calib/' + camera_name + '.yaml'
        with open(filename, mode='w') as file:
            yaml.dump(data, file, default_flow_style=False)
            rospy.loginfo('Saved transform from camera base frame[%s] to camera parent frame[%s] into %s'
                          % (camera_base_frame, camera_parent_frame, filename))


######################################################################
#  global functions                                                  #
######################################################################
if __name__ == '__main__':
    rospy.init_node('run_volume_calibration')

    calibration = CameraCalibrationRoutines()
    calibration.run()
