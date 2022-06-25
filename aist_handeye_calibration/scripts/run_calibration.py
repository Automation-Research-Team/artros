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
import rospy, copy, actionlib
from math                         import radians
from std_srvs.srv                 import Empty, Trigger
from geometry_msgs                import msg as gmsg
from tf                           import transformations as tfs
from aist_handeye_calibration.srv import GetSampleList, ComputeCalibration
from aist_routines                import AISTBaseRoutines
from aist_handeye_calibration.msg import TakeSampleAction, TakeSampleGoal
from actionlib_msgs.msg           import GoalStatus

######################################################################
#  class HandEyeCalibrationRoutines                                  #
######################################################################
class HandEyeCalibrationRoutines(AISTBaseRoutines):
    def __init__(self):
        super(HandEyeCalibrationRoutines, self).__init__()

        self._camera_name          = rospy.get_param('~camera_name',
                                                    'realsenseD435')
        self._robot_name           = rospy.get_param('~robot_name', 'b_bot')
        self._eye_on_hand          = rospy.get_param('~eye_on_hand', False)
        self._robot_base_frame     = rospy.get_param('~robot_base_frame',
                                                    'workspace_center')
        self._robot_effector_frame = rospy.get_param('~robot_effector_frame',
                                                    'b_bot_ee_link')
        self._initpose             = rospy.get_param('~initpose', [])
        self._keyposes             = rospy.get_param('~keyposes', [])
        self._speed                = rospy.get_param('~speed', 1)
        self._sleep_time           = rospy.get_param('~sleep_time', 0.5)

        if rospy.get_param('calibration', True):
            ns = '/handeye_calibrator'
            self.get_sample_list = rospy.ServiceProxy(ns + '/get_sample_list',
                                                    GetSampleList)
            self.compute_calibration = rospy.ServiceProxy(
                ns + '/compute_calibration', ComputeCalibration)
            self.save_calibration = rospy.ServiceProxy(ns + '/save_calibration',
                                                    Trigger)
            self.reset = rospy.ServiceProxy(ns + '/reset', Empty)
            self.take_sample = actionlib.SimpleActionClient(ns + '/take_sample',
                                                            TakeSampleAction)

        else:
            self.get_sample_list     = None
            self.compute_calibration = None
            self.save_calibration    = None
            self.reset               = None
            self.take_sample         = None

    def move(self, pose):
        poseStamped = gmsg.PoseStamped()
        poseStamped.header.frame_id = self._robot_base_frame
        poseStamped.pose = gmsg.Pose(gmsg.Point(*pose[0:3]),
                                     gmsg.Quaternion(
                                         *tfs.quaternion_from_euler(
                                             *map(radians, pose[3:6]))))
        print('  move to ' + self.format_pose(poseStamped))
        (success, _, current_pose) \
            = self.go_to_pose_goal(
                self._robot_name, poseStamped, self._speed,
                end_effector_link=self._robot_effector_frame,
                move_lin=True)
        print('  reached ' + self.format_pose(current_pose))
        return success

    def move_to(self, pose, keypose_num, subpose_num):
        if not self.move(pose):
            return False

        if self.take_sample:
            rospy.sleep(self._sleep_time)  # Wait for the robot to settle.
            self.take_sample.send_goal(TakeSampleGoal())
            self.trigger_frame(self._camera_name)
            if not self.take_sample.wait_for_result(rospy.Duration(5.0)):
                self.take_sample.cancel_goal()  # timeout expired
                rospy.logerr('TakeSampleAction: timeout expired')
                return False
            if self.take_sample.get_state() != GoalStatus.SUCCEEDED:
                rospy.logerr('TakeSampleAction: not in succeeded state')
                return False

            result = self.take_sample.get_result()
            pose = gmsg.PoseStamped()
            pose.header = result.cMo.header
            pose.pose.position    = result.cMo.transform.translation
            pose.pose.orientation = result.cMo.transform.rotation
#            print('  camera <= obejct   ' + self.format_pose(pose))
            pose.header = result.wMe.header
            pose.pose.position    = result.wMe.transform.translation
            pose.pose.orientation = result.wMe.transform.rotation
            print('  world  <= effector ' + self.format_pose(pose))

            n = len(self.get_sample_list().cMo)
            print('  {} samples taken').format(n)

        return True

    def move_to_subposes(self, pose, keypose_num):
        subpose = copy.copy(pose)
        roll = subpose[3]
        for i in range(3):
            print('\n--- Subpose [{}/5]: Try! ---'.format(i + 1))
            if self.move_to(subpose, keypose_num, i + 1):
                print('--- Subpose [{}/5]: Succeeded. ---'.format(i + 1))
            else:
                print('--- Subpose [{}/5]: Failed. ---'.format(i + 1))
            subpose[3] -= 30

        subpose[3] = roll - 30
        subpose[4] += 15

        for i in range(2):
            print('\n--- Subpose [{}/5]: Try! ---'.format(i + 4))
            if self.move_to(subpose, keypose_num, i + 4):
                print('--- Subpose [{}/5]: Succeeded. ---'.format(i + 4))
            else:
                print('--- Subpose [{}/5]: Failed. ---'.format(i + 4))
            subpose[4] -= 30

    def calibrate(self):
        self.continuous_shot(self._camera_name, False)

        if self.reset:
            self.reset()

        # Reset pose
        self.go_to_named_pose(self._robot_name, 'home')
        self.move(self._initpose)

        # Collect samples over pre-defined poses
        keyposes = self._keyposes
        for i, keypose in enumerate(keyposes, 1):
            print('\n*** Keypose [{}/{}]: Try! ***'
                  .format(i, len(keyposes)))
            if self._eye_on_hand:
                self.move_to(keypose, i, 1)
            else:
                self.move_to_subposes(keypose, i)
            print('*** Keypose [{}/{}]: Completed. ***'
                  .format(i, len(keyposes)))

        if self.compute_calibration:
            try:
                res = self.compute_calibration()
                print(res.message)
                if res.success:
                    res = self.save_calibration()
                    print(res.message)
            except rospy.ServiceException as e:
                rospy.logerr('Service call failed: %s' % e)

        self.go_to_named_pose(self._robot_name, 'home')

    def run(self):
        while not rospy.is_shutdown():
            print('\n  RET: do calibration')
            print('  q  : go to home position and quit')
            if raw_input('>> ') == 'q':
                break
            self.calibrate()


######################################################################
#  global functions                                                  #
######################################################################
if __name__ == '__main__':
    rospy.init_node('run_calibration')

    with HandEyeCalibrationRoutines() as calibrate:
        calibrate.run()
