#!/usr/bin/env python

import sys
import os
import rospy
import argparse
from math          import radians, degrees
from std_srvs.srv  import Empty, Trigger
from geometry_msgs import msg as gmsg
from tf            import TransformListener, transformations as tfs
from aist_routines import AISTBaseRoutines



######################################################################
#  class FTCalibrationRoutines                                       #
######################################################################
class FTCalibrationRoutines(AISTBaseRoutines):
    def __init__(self, needs_calib=True):
        super(FTCalibrationRoutines, self).__init__()

        self._ftsensor_name        = rospy.get_param('~ftsensor_name',
                                                     'a_bot_ftsensor')
        self._robot_name           = rospy.get_param('~robot_name', 'a_bot')
        self._speed                = rospy.get_param('~speed',       0.1)
        self._robot_base_frame     = rospy.get_param('~robot_base_frame',
                                                     'workspace_center')
        self._robot_effector_frame = rospy.get_param('~robot_effector_frame',
                                                     'a_bot_ee_link')
        self._initpose             = rospy.get_param('~initpose', [])

        if needs_calib:
            ns = self._ftsensor_name + '/wrench/'
            self.take_sample = rospy.ServiceProxy(ns + 'take_sample', Trigger)
            self.compute_calibration = rospy.ServiceProxy(
                ns + 'compute_calibration', Trigger)
            self.save_calibration = rospy.ServiceProxy(ns + 'save_calibration',
                                                       Trigger)
            self.clear_samples = rospy.ServiceProxy(ns + 'clear_samples',
                                                    Trigger)
        else:
            self._take_sample         = None
            self._compute_calibration = None
            self._save_calibration    = None

    def move(self, xyzrpy):
        target_pose = gmsg.PoseStamped()
        target_pose.header.frame_id = self.reference_frame
        target_pose.pose = gmsg.Pose(gmsg.Point(*xyzrpy[0:3]),
                                     gmsg.Quaternion(
                                         *tfs.quaternion_from_euler(
                                             *map(radians, xyzrpy[3:6]))))
        success, _, _ = self.go_to_pose_goal(self._robot_name, target_pose,
                                             self._speed, move_lin=True)
        return success

    def calibrate(self):
        self.go_to_named_pose(self._robot_name, 'home')
        if self.clear_samples:
            self.clear_samples()

        self.move(self._initpose)
        if self.take_sample:
            self.take_sample()

        max_angle = 45
        step = 5
        for i in range(max_angle/step):
            self.move_relative(self._robot_name,
                               [0, 0, 0], [0, radians(step), 0], self._speed)
            if self.take_sample:
                self.take_sample()

        self.move_relative(self._robot_name,
                           [0, 0, 0], [0, radians(-max_angle), 0], self._speed)

        for i in range(max_angle/step):
            self.move_relative(self._robot_name,
                               [0, 0, 0], [0, radians(-step), 0], self._speed)
            if self.take_sample:
                self.take_sample()

        self.move_relative(self._robot_name,
                           [0, 0, 0], [0, radians(max_angle), 0], self._speed)

        for i in range(max_angle/step):
            self.move_relative(self._robot_name,
                               [0, 0, 0], [0, 0, radians(step)], self._speed)
            if self.take_sample:
                self.take_sample()

        self.move_relative(self._robot_name,
                           [0, 0, 0], [0, 0, radians(-max_angle)], self._speed)

        for i in range(max_angle/step):
            self.move_relative(self._robot_name,
                               [0, 0, 0], [0, 0, radians(-step)], self._speed)
            if self.take_sample:
                self.take_sample()

        self.move_relative(self._robot_name,
                           [0, 0, 0], [0, 0, radians(max_angle)], self._speed)

        if self.compute_calibration:
            res = self.compute_calibration()
            print('  compute calibration: {}').format(res.message)
            res = self.save_calibration()
            print('  save calibration: {}').format(res.message)

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

    rospy.init_node('calib_ftsensor', anonymous=True)

    with FTCalibrationRoutines() as routines:
        routines.run()
