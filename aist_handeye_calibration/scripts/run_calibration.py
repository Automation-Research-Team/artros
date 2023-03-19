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
from std_srvs.srv                 import Empty, Trigger
from geometry_msgs.msg            import PoseStamped, Pose, Point, Quaternion
from actionlib_msgs.msg           import GoalStatus
from tf                           import transformations as tfs
from aist_routines                import AISTBaseRoutines
from aist_handeye_calibration.srv import GetSampleList, ComputeCalibration
from aist_handeye_calibration.msg import TakeSampleAction, TakeSampleGoal

######################################################################
#  class HandEyeCalibrationRoutines                                  #
######################################################################
class HandEyeCalibrationRoutines(AISTBaseRoutines):
    def __init__(self):
        super(HandEyeCalibrationRoutines, self).__init__()

        self._camera_name          = rospy.get_param('~camera_name',
                                                     'a_phoxi_m_camera')
        self._robot_name           = rospy.get_param('~robot_name', 'b_bot')
        self._eye_on_hand          = rospy.get_param('~eye_on_hand', False)
        self._robot_base_frame     = rospy.get_param('~robot_base_frame',
                                                     'workspace_center')
        self._robot_effector_frame = rospy.get_param('~robot_effector_frame',
                                                     'b_bot_flange')
        self._robot_effector_tip_frame \
                = rospy.get_param('~robot_effector_tip_frame', '')
        self._initpose             = rospy.get_param('~initpose', [])
        self._keyposes             = rospy.get_param('~keyposes', [])
        self._speed                = rospy.get_param('~speed', 1)
        self._sleep_time           = rospy.get_param('~sleep_time', 0.5)

        if rospy.get_param('calibration', True):
            ns = '/handeye_calibrator'
            self._get_sample_list     = rospy.ServiceProxy(
                                            ns + '/get_sample_list',
                                            GetSampleList)
            self._compute_calibration = rospy.ServiceProxy(
                                            ns + '/compute_calibration',
                                            ComputeCalibration)
            self._save_calibration    = rospy.ServiceProxy(
                                            ns + '/save_calibration', Trigger)
            self._reset               = rospy.ServiceProxy(ns + '/reset',
                                                           Empty)
            self._take_sample         = actionlib.SimpleActionClient(
                                            ns + '/take_sample',
                                            TakeSampleAction)
        else:
            self._get_sample_list     = None
            self._compute_calibration = None
            # self._save_calibration    = None
            self._reset               = None
            self._take_sample         = None

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
        super(HandEyeCalibrationRoutines, self).print_help_messages()
        print('=== Calibration commands ===')
        print('  init:  go to initial pose')
        print('  calib: do calibration')
        print('  RET:   go to marker')

    def interactive(self, key, robot_name, axis, speed):
        if key == 'init':
            self.go_to_initpose()
        elif key == 'calib':
            self.calibrate()
        elif key == '':
            self.go_to_marker()
        else:
            return super(HandEyeCalibrationRoutines, self) \
                  .interactive(key, robot_name, axis, speed)
        return robot_name, axis, speed

    def go_to_initpose(self):
        self._move(self._initpose, self._robot_effector_frame)

    def calibrate(self):
        self.continuous_shot(self._camera_name, False)

        if self._reset:
            self._reset()

        # Reset pose
        self.go_to_named_pose(self._robot_name, 'home')
        self.go_to_initpose()

        # Collect samples over pre-defined poses
        keyposes = self._keyposes
        for i, keypose in enumerate(keyposes, 1):
            print('\n*** Keypose [{}/{}]: Try! ***'.format(i, len(keyposes)))
            if self._eye_on_hand:
                self._move_to(keypose, i, 1)
            else:
                self._move_to_subposes(keypose, i)
            print('*** Keypose [{}/{}]: Completed. ***'
                  .format(i, len(keyposes)))

        if self._compute_calibration:
            try:
                res = self._compute_calibration()
                print(res.message)
                if res.success:
                    self._save_camera_placement(res.eMc)
                    res = self._save_calibration()
                    print(res.message)
            except rospy.ServiceException as e:
                rospy.logerr('Service call failed: %s' % e)
            except Exception as e:
                rospy.logerr(e)
        self.go_to_named_pose(self._robot_name, 'home')

    def go_to_marker(self):
        self.trigger_frame(self._camera_name)
        marker_pose = rospy.wait_for_message('/aruco_detector/pose',
                                             PoseStamped, 10)
        approach_pose = self.effector_target_pose(marker_pose, (0, 0, 0.05))

        # We have to transform the target pose to reference frame before moving
        # to the approach pose because the marker pose is given w.r.t. camera
        # frame which will change while moving in the case of "eye on hand".
        target_pose = self.transform_pose_to_target_frame(
                        self.effector_target_pose(marker_pose, (0, 0, 0)))
        print('  move to ' + self.format_pose(approach_pose))
        success, _, current_pose \
            = self.go_to_pose_goal(
                self._robot_name, approach_pose, self._speed,
                end_effector_link=self._robot_effector_tip_frame,
                move_lin=True)
        print('  reached ' + self.format_pose(current_pose))
        rospy.sleep(1)
        print('  move to ' + self.format_pose(target_pose))
        success, _, current_pose \
            = self.go_to_pose_goal(
                self._robot_name, target_pose, 0.05,
                end_effector_link=self._robot_effector_tip_frame,
                move_lin=True)
        print('  reached ' + self.format_pose(current_pose))

    # Move stuffs
    def _move_to_subposes(self, keypose, keypose_num):
        subpose = copy.copy(keypose)
        roll = subpose[3]
        for i in range(3):
            print('\n--- Subpose [{}/5]: Try! ---'.format(i + 1))
            if self._move_to(subpose, keypose_num, i + 1):
                print('--- Subpose [{}/5]: Succeeded. ---'.format(i + 1))
            else:
                print('--- Subpose [{}/5]: Failed. ---'.format(i + 1))
            subpose[3] -= 30

        subpose[3] = roll - 30
        subpose[4] += 15

        for i in range(2):
            print('\n--- Subpose [{}/5]: Try! ---'.format(i + 4))
            if self._move_to(subpose, keypose_num, i + 4):
                print('--- Subpose [{}/5]: Succeeded. ---'.format(i + 4))
            else:
                print('--- Subpose [{}/5]: Failed. ---'.format(i + 4))
            subpose[4] -= 30

    def _move_to(self, subpose, keypose_num, subpose_num):
        if not self._move(subpose, self._robot_effector_frame):
            return False

        if self._take_sample:
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
            pose = PoseStamped()
            pose.header = result.cMo.header
            pose.pose.position    = result.cMo.transform.translation
            pose.pose.orientation = result.cMo.transform.rotation
#            print('  camera <= obejct   ' + self.format_pose(pose))
            pose.header = result.wMe.header
            pose.pose.position    = result.wMe.transform.translation
            pose.pose.orientation = result.wMe.transform.rotation
            print('  world  <= effector ' + self.format_pose(pose))

            n = len(self._get_sample_list().cMo)
            print('  {} samples taken').format(n)

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

    def _save_camera_placement(self, eMc):
        # Frame to which the camera attached
        camera_parent_frame = rospy.get_param('~camera_parent_frame')

        # Get camera base frame whose parent is camera_parent_frame.
        camera_frame      = eMc.child_frame_id
        stamp             = eMc.header.stamp
        chain             = self.listener.chain(camera_parent_frame, stamp,
                                                camera_frame, stamp,
                                                camera_parent_frame)
        camera_base_frame = chain[-2]

        # Compute transform from camera base frame to its parent.
        eTc = self.listener.fromTranslationRotation(
                                (eMc.transform.translation.x,
                                 eMc.transform.translation.y,
                                 eMc.transform.translation.z),
                                (eMc.transform.rotation.x,
                                 eMc.transform.rotation.y,
                                 eMc.transform.rotation.z,
                                 eMc.transform.rotation.w))
        pTe = self.listener.fromTranslationRotation(
                                *self.listener.lookupTransform(
                                    camera_parent_frame,
                                    eMc.header.frame_id, stamp))
        cTb = self.listener.fromTranslationRotation(
                                *self.listener.lookupTransform(
                                    camera_frame, camera_base_frame, stamp))
        pTb = tfs.concatenate_matrices(pTe, eTc, cTb)

        # Convert the transform to xyz-rpy representation.
        xyz  = map(float, tfs.translation_from_matrix(pTb))
        rpy  = map(float, tfs.euler_from_matrix(pTb))
        data = {'parent': camera_parent_frame,
                'child' : camera_base_frame,
                'origin': xyz + rpy}
        print(data)
        # Save the transform.
        filename = rospkg.RosPack().get_path('aist_handeye_calibration') \
                 + '/calib/' + self._camera_name + '.yaml'
        with open(filename, mode='w') as file:
            yaml.dump(data, file, default_flow_style=False)
            rospy.loginfo('Saved transform from camera base frame[%s] to camera parent frame[%s] into %s'
                          % (camera_base_frame, camera_parent_frame, filename))


######################################################################
#  global functions                                                  #
######################################################################
if __name__ == '__main__':
    rospy.init_node('run_calibration')

    calibration = HandEyeCalibrationRoutines()
    calibration.run()
