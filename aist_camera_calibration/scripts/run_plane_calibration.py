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
from tf                          import (TransformListener,
                                         transformations as tfs)
from aist_camera_calibration.srv import GetSampleList, ComputeCalibration
from aist_camera_calibration.msg import TakeSampleAction, TakeSampleGoal
from aist_utility.compat         import *

######################################################################
#  class CameraCalibrationRoutines                                  #
######################################################################
class CameraCalibrationRoutines(object):
    def __init__(self):
        super(CameraCalibrationRoutines, self).__init__()

        ns = '/camera_calibrator'
        self._listener            = TransformListener()
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
        while not rospy.is_shutdown():
            print('\n  q  : quit program')
            print('  RET: take sample')
            print('  g  : get sample list')
            print('  c  : compute calibration')
            print('  s  : save calibration')
            print('  r  : reset and discard all samples')

            prompt = '>> '
            key = raw_input(prompt)
            if key == 'q':
                break
            elif key == 'g':
                self.get_sample_list()
            elif key == 'c':
                self.compute_calibration()
            elif key == 'r':
                self._reset()
            else:
                self.take_sample()

    def take_sample(self):
        self._take_sample.send_goal(TakeSampleGoal())
        if not self._take_sample.wait_for_result(rospy.Duration(5.0)):
            self._take_sample.cancel_goal()  # timeout expired
            rospy.logerr('TakeSampleAction: timeout expired')
            return False
        if self._take_sample.get_state() != GoalStatus.SUCCEEDED:
            rospy.logerr('TakeSampleAction: not in SUCCEEDED state')
            return False
        result = self._take_sample.get_result()

        rospy.loginfo('TakeSampleAction: SUCCEEDED')
        for correspondences in result.correspondences_set:
            print('  [%s] %d point correspondences w.r.t. %s'
                  % (correspondences.camera_name,
                     len(correspondences.correspondences),
                     correspondences.reference_frame))

    def get_sample_list(self):
        res = self._get_sample_list()
        rospy.loginfo('GetSampleList: %d samples obtained'
                      % len(res.correspondences_sets))
        for correspondences_set in res.correspondences_sets:
            for correspondences in correspondences_set.correspondences_set:
                print('  [%s] %d point correspondences w.r.t. %s'
                      % (correspondences.camera_name,
                         len(correspondences.correspondences),
                         correspondences.reference_frame))
            print('')

    def compute_calibration(self):
        res = self._compute_calibration()
        if not res.success:
            rospy.logerr('ComputeCalibration: failed')
            return

        for camera_name, intrinsic, pose in zip(res.camera_names,
                                                res.intrinsics, res.poses):
            self._save_camera_pose(camera_name, intrinsic, pose)
        rospy.loginfo('ComputeCalibration: succeeded with reprojection err[%f]'
                      % res.error)
        res = self._save_calibration()
        if res.success:
            rospy.loginfo('SaveCalibration: succeeded')
        else:
            rospy.logerr('SaveCalibration: failed')

    def _save_camera_pose(self, camera_name, intrinsic, pose):
        # Frame to which the camera attached
        camera_parent_frame = rospy.get_param('~camera_parent_frame', '')
        if camera_parent_frame == '':
            return

        # Get camera base frame whose parent is camera_parent_frame.
        camera_frame      = intrinsic.header.frame_id
        stamp             = intrinsic.header.stamp
        reference_frame   = pose.header.frame_id
        chain             = self._listener.chain(camera_parent_frame, stamp,
                                                 camera_frame, stamp,
                                                 camera_parent_frame)
        camera_base_frame = chain[-2]

        # Compute transform from camera base frame to its parent.
        rTc = self._listener.fromTranslationRotation((pose.pose.position.x,
                                                     pose.pose.position.y,
                                                     pose.pose.position.z),
                                                    (pose.pose.orientation.x,
                                                     pose.pose.orientation.y,
                                                     pose.pose.orientation.z,
                                                     pose.pose.orientation.w))
        pTr = self._listener.fromTranslationRotation(
                                 *self._listener.lookupTransform(
                                     camera_parent_frame,
                                     reference_frame, stamp))
        cTb = self._listener.fromTranslationRotation(
                                 *self._listener.lookupTransform(
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
    rospy.init_node('run_plane_calibration')

    calibration = CameraCalibrationRoutines()
    calibration.run()
