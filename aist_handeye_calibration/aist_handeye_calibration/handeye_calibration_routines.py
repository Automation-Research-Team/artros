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
import rclpy, yaml
from rclpy.callback_groups     import MutuallyExclusiveCallbackGroup
from aist_routines             import BaseRoutines
from std_srvs.srv              import Empty
from aist_msgs.srv             import (HandEyeCalibrationTakeSample,
                                       HandEyeCalibrationGetSampleList,
                                       HandEyeCalibrationComputeCalibration)
from task_wrappers             import ServiceClient
from aist_utility.fileio       import filepath_from_url
from .handeye_calibration_task import HandEyeCalibrationTask

#*********************************************************************
#  class HandEyeCalibrationRoutines                                  *
#*********************************************************************
class HandEyeCalibrationRoutines(BaseRoutines):
    def __init__(self, name, calibrator_ns='handeye_calibrator',
                 task_ns='handeye_calibration'):
        super().__init__(name)

        self._camera_name       = self.declare_parameter('camera_name',
                                                         'a_motioncam').value
        self._robot_name        = self.declare_parameter('robot_name',
                                                         'b_bot').value
        self._eye_on_hand       = self.declare_parameter('eye_on_hand',
                                                         False).value
        self._end_effector_link = self.declare_parameter('end_effector_link',
                                                         'b_bot_flange').value
        self._calib_file        = self.declare_parameter('calibration_file',
                                                         '').value
        self._speed             = self.declare_parameter('speed', 1.0).value
        self._settling_time     = self.declare_parameter('settling_time',
                                                         2.0).value
        self._initpose          = self.declare_parameter('initpose',
                                                         [0.0]).value
        self._keyposes          = self.declare_parameter('keyposes',
                                                         [0.0]).value

        self._cbg                 = MutuallyExclusiveCallbackGroup()
        self._take_sample         = ServiceClient(
                                        self,
                                        HandEyeCalibrationTakeSample,
                                        calibrator_ns + '/take_sample',
                                        callback_group=self._cbg)
        self._get_sample_list     = ServiceClient(
                                        self,
                                        HandEyeCalibrationGetSampleList,
                                        calibrator_ns + '/get_sample_list',
                                        callback_group=self._cbg)
        self._compute_calibration = ServiceClient(
                                        self,
                                        HandEyeCalibrationComputeCalibration,
                                        calibrator_ns + '/compute_calibration',
                                        callback_group=self._cbg)
        self._reset               = ServiceClient(
                                        self,
                                        Empty, calibrator_ns + '/reset',
                                        callback_group=self._cbg)
        self._handeye_calibration = HandEyeCalibrationTask(self, task_ns)

    # interactive stuffs
    def do_cmds(self, dummy):
        """      Print command list."""
        super().do_cmds(dummy)
        print('=== Calibration commands ===')
        print('  calib:   do calibration')
        print('  ccancel: cancel calibration and then return to home pose')
        print('  check:   go to marker')
        print('  clist:   get list of sample points')
        print('  creset:  discard all sample potins')

    def do_calib(self, dummy):
        """      calib
        Do hand-eye calibration"""
        self.calibrate(timeout_sec=0.0)

    def do_ccancel(self, dummy):
        """      ccancel
        Cancel calibration and return to home pose"""
        self._handeye_calibration.cancel()

    def do_check(self, dummy):
        """      check
        Go to marker with calibrated camera"""
        self.go_to_marker()

    def do_clist(self, dummy):
        """       clist
        Print list of sample points"""
        print(self.get_sample_list())

    def do_creset(self, dummy):
        """      creset
        Discard all sample points"""
        self.reset()

    # calibration stuffs
    def calibrate(self, *, timeout_sec=None):
        self._handeye_calibration.send_goal(self._camera_name,
                                            self._robot_name,
                                            self._eye_on_hand,
                                            self._end_effector_link,
                                            self._initpose,
                                            self._keyposes,
                                            timeout_sec=timeout_sec)

    def go_to_marker(self):
        self.trigger_frame(self._camera_name)
        _, marker_pose = rclpy.wait_for_message(PoseStamped, self,
                                                '/detector_3d/pose',
                                                time_to_wait=2.0)
        if marker_pose is None:
            self.get_logger().error('failed to detect marker')
            return False
        marker_pose = self.transform_pose_to_target_frame(marker_pose)
        success = self.go_to_pose_goal(self._robot_name,
                                       marker_pose, (0.0, 0.0, 0.05),
                                       speed=self._speed)
        if success:
            self.get_logger().info('  reached approach pose: %s' %
                                   self.format_pose(self.get_current_pose(
                                                        self._robot_name)))
        else:
            self.get_logger().error('  failed to reach approach pose: %s' %
                                    self.format_pose(marker_pose))

        time.sleep(1.0)

        success = self.go_to_pose_goal(self._robot_name,
                                       marker_pose, speed=0.05)
        if success:
            self.get_logger().info('  reached marker pose: %s' %
                                   self.format_pose(self.get_current_pose(
                                                        self._robot_name)))
        else:
            self.get_logger().error('  failed to reach marker pose: %s' %
                                    self.format_pose(marker_pose))

    def take_sample(self, timeout_sec=None):
        self._take_sample.call(HandEyeCalibrationTakeSample.Request(),
                               timeout_sec=timeout_sec)

    def wait_for_sample(self, timeout_sec=None):
        return self._take_sample.wait(timeout_sec=timeout_sec)

    def get_sample_list(self):
        return self._get_sample_list.call(HandEyeCalibrationGetSampleList\
                                          .Request())

    def compute_calibration(self):
        return self._compute_calibration.call(
                   HandEyeCalibrationComputeCalibration.Request())

    def reset(self):
        return self._reset.call(Empty.Request())

    def save_calibration(self, res):
        def xyzrpy_from_transform(transform):
            rpy = tfs.euler_from_quaternion((transform.rotation.x,
                                             transform.rotation.y,
                                             transform.rotation.z,
                                             transform.rotation.w))
            return [transform.translation.x,
                    transform.translation.y,
                    transform.translation.z,
                    degrees(rpy[0]), degrees(rpy[1]), degrees(rpy[2])]

        print('=== estimated camera pose ===')
        print('[{:.4f}, {:.4f}, {:.4f}; {:.2f}, {:.2f}. {:.2f}]'\
              .format(*xyzrpy_from_transform(res.transform_ec.transform)))
        print('=== estimated marker pose ===')
        print('[{:.4f}, {:.4f}, {:.4f}; {:.2f}, {:.2f}. {:.2f}]'\
              .format(*xyzrpy_from_transform(res.transform_wm.transform)))
        print('trans. err(m): (mean, max) = (%f, %f)'
              % (res.mean_translation_error, res.max_translation_error))
        print('rot. err(deg): (mean, max) = (%f, %f)'
              % (res.mean_rotation_error, res.max_rotation_error))

        # Convert the transform to xyz-rpy representation.
        data = {'parent': res.transform_ec.header.frame_id,
                'child' : res.transform_ec.child_frame_id,
                'origin': xyzrpy_from_transform(res.transform_ec.transform)}

        # Save the transform.
        filename = filepath_from_url(self._calib_file)
        with open(filename, mode='w') as file:
            yaml.dump(data, file, default_flow_style=False)
        self.get_logger().info('saved calibration result in [%s]' % filename)
