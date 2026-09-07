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
import rclpy
from rclpy.callback_groups        import MutuallyExclusiveCallbackGroup
from aist_routines                import BaseRoutines
from std_srvs.srv                 import Empty
from aist_msgs.srv                import (CameraCalibrationTakeSample,
                                          CameraCalibrationGetSampleList,
                                          CameraCalibrationComputeCalibration)
from task_wrappers.service_client import ServiceClient
from .camera_calibration_task     import CameraCalibrationTask
from .utilities                   import dict_from_point_correspondences_sets

#*********************************************************************
#  class CameraCalibrationRoutines                                   *
#*********************************************************************
class CameraCalibrationRoutines(BaseRoutines):
    def __init__(self, name, calibrator_ns='camera_calibrator',
                 task_ns='camera_calibration'):
        super().__init__(name)

        self._robot_name        = self.declare_parameter('robot_name',
                                                         'b_bot').value
        self._end_effector_link = self.declare_parameter('end_effector_link',
                                                         'b_bot_flange').value
        self._calib_dir         = self.declare_parameter('calibration_dir',
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
                                        CameraCalibrationTakeSample,
                                        calibrator_ns + '/take_sample',
                                        callback_group=self._cbg)
        self._get_sample_list     = ServiceClient(
                                        self,
                                        CameraCalibrationGetSampleList,
                                        calibrator_ns + '/get_sample_list',
                                        callback_group=self._cbg)
        self._compute_calibration = ServiceClient(
                                        self,
                                        CameraCalibrationComputeCalibration,
                                        calibrator_ns + '/compute_calibration',
                                        callback_group=self._cbg)
        self._reset               = ServiceClient(
                                        self,
                                        Empty, calibrator_ns + '/reset',
                                        callback_group=self._cbg)
        self._camera_calibration  = CameraCalibrationTask(self, task_ns)

    # interactive stuffs
    def do_cmds(self, dummy):
        """      Print command list."""
        super().do_cmds(dummy)
        print('=== Calibration commands ===')
        print('  calib:   do calibration')
        print('  ccancel: cancel calibration and then return to home pose')
        print('  clist:   get list of sample points')
        print('  ctake:   take sample points')
        print('  creset:  discard all sample potins')

    def do_calib(self, dummy):
        """      calib
        Do hand-eye calibration"""
        self.calibrate(timeout_sec=0.0)

    def do_ccancel(self, dummy):
        """      ccancel
        Cancel calibration and return to home pose"""
        self._camera_calibration.cancel()

    def do_clist(self, dummy):
        """       clist
        Print list of sample points"""
        res = self.get_sample_list()
        self.get_logger().info(res.message)
        for correspondences_set in res.correspondences_sets:
            for correspondences in correspondences_set.correspondences_set:
                print('  [%s] %d point correspondences w.r.t. %s'
                      % (correspondences.camera_name,
                         len(correspondences.correspondences),
                         correspondences.reference_frame))
            print('')

    def do_ctake(self, dummy):
        """      ctake
        Take sample points and print them"""
        status, result = self.wait_for_samle(
                     self._calibrator.take_sample_async())
        self.get_logger().info(result.message)
        for correspondences in result.correspondences_set:
            print('  [%s] %d point correspondences w.r.t. %s'
                  % (correspondences.camera_name,
                     len(correspondences.correspondences),
                     correspondences.reference_frame))

    def do_creset(self, dummy):
        """      creset
        Discard all sample points"""
        self.reset()

    # calibration stuffs
    def calibrate(self, *, timeout_sec=None):
        self._camera_calibration.send_goal(self._camera_name,
                                            self._robot_name,
                                            self._eye_on_hand,
                                            self._end_effector_link,
                                            self._initpose,
                                            self._keyposes,
                                            timeout_sec=timeout_sec)

    def take_sample(self, timeout_sec=None):
        self._take_sample.call(CameraCalibrationTakeSample.Request(),
                               timeout_sec=timeout_sec)

    def wait_for_sample(self, timeout_sec=None):
        return self._take_sample.wait(timeout_sec)

    def get_sample_list(self):
        return self._get_sample_list.call(CameraCalibrationGetSampleList\
                                          .Request())

    def compute_calibration(self):
        return self._compute_calibration.call(
                   CameraCalibrationComputeCalibration.Request())

    def reset(self):
        return self._reset.call(Empty.Request())

    def save_calibration(self, res):
        for camera_name, camera_info, camera_pose in zip(res.camera_names,
                                                         res.intrinsics,
                                                         res.camera_poses):
            print('=== estimated pose of %s ===' % camera_name)
            print('[{:.4f}, {:.4f}, {:.4f}; {:.2f}, {:.2f}. {:.2f}]'\
                  .format(*self.xyzrpy_from_pose(camera_pose)))

            # Convert camera pose to xyz-rpy representation.
            data = {'parent': camera_pose.header.frame_id,
                    'child' : camera_info.header.frame_id,
                    'origin': [float(t) for t in self.xyzrpy_from_pose(
                                                     camera_pose)]}

            # Save camera pose.
            dirname  = filepath_from_url(self._calib_dir)
            filename = dirname + '/' + camera_name + '.yaml'
            with open(filename, mode='w') as f:
                yaml.dump(data, f, default_flow_style=False)
            self.get_logger().info('saved camera extrinsiscs in [%s]'
                                   % filename)

            # Save camera_info.
            filename = dirname + '/' + camera_name + '-camera_info.yaml'
            with open(filename, mode='w') as f:
                yaml.dump(dict_from_camera_info(camera_name, camera_info),
                          f, default_flow_style=False)
            self.get_logger().info(
                'saved camera intrinsiscs in [%s]' % filename)

        print('=== reprojection error: %f(pix) ===' % res.error)
