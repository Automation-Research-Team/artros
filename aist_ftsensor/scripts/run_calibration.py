#!/usr/bin/env python

import rospy
from math              import radians
from std_srvs.srv      import Trigger
from geometry_msgs.msg import PoseStamed, Pose, point, Quaternion
from tf                import transformations as tfs
from aist_routines     import AISTBaseRoutines

######################################################################
#  class FTCalibrationRoutines                                       #
######################################################################
class FTCalibrationRoutines(AISTBaseRoutines):
    def __init__(self, needs_calib=True):
        super(FTCalibrationRoutines, self).__init__()

        self._robot_name = rospy.get_param('~robot_name', 'a_bot')
        self._speed      = rospy.get_param('~speed',       0.1)
        self._initpose   = rospy.get_param('~initpose',    [])

        if needs_calib:
            ftsensor_name = rospy.get_param('~ftsensor_name', 'a_bot_ftsensor')
            ns = ftsensor_name + '/wrench'
            self._take_sample         = rospy.ServiceProxy(
                                          ns + '/take_sample', Trigger)
            self._compute_calibration = rospy.ServiceProxy(
                                          ns + '/compute_calibration', Trigger)
            self._save_calibration    = rospy.ServiceProxy(
                                          ftsensor_name + '/save_calibration',
                                          Trigger)
            self._clear_samples       = rospy.ServiceProxy(
                                          ns + '/clear_samples', Trigger)
        else:
            self._take_sample         = None
            self._compute_calibration = None
            self._save_calibration    = None
            self._clear_samples       = None

    def run(self):
        self.print_help_messages()
        print('')

        axis = 'Y'

        while not rospy.is_shutdown():
            prompt = '{:>5}:{}>> ' \
                     .format(axis,
                             self.format_pose(
                                 self.get_current_pose(self._robot_name)))
            key = raw_input(prompt)

            try:
                _, axis, _ = self.interactive(key, self._robot_name, axis,
                                              self._speed)
            except Exception as e:
                print(e.message)

    # Interactive stuffs
    def print_help_messages(self):
        super(FTCalibrationRoutines, self).print_help_messages()
        print('=== Calibration commands ===')
        print('  RET: do calibration')

    def interactive(self, key, robot_name, axis, speed):
        if key == '':
            self.calibrate()
        else:
            return super(FTCalibrationRoutines, self) \
                  .interactive(key, robot_name, axis, speed)
        return robot_name, axis, speed

    # Commands
    def calibrate(self):
        self.go_to_named_pose(self._robot_name, 'home')
        if self._clear_samples:
            self._clear_samples()

        self.go_to_pose_goal(self._robot_name,
                             self.pose_from_xyzrpy(self._initpoe), self._speed)
        if self._take_sample:
            self._take_sample()

        max_angle = 45
        step = 5
        for i in range(max_angle/step):
            self.move_relative(self._robot_name,
                               [0, 0, 0, 0, radians(step), 0], self._speed)
            if self._take_sample:
                self._take_sample()

        self.move_relative(self._robot_name,
                           [0, 0, 0, 0, radians(-max_angle), 0], self._speed)

        for i in range(max_angle/step):
            self.move_relative(self._robot_name,
                               [0, 0, 0, 0, radians(-step), 0], self._speed)
            if self._take_sample:
                self._take_sample()

        self.move_relative(self._robot_name,
                           [0, 0, 0, 0, radians(max_angle), 0], self._speed)

        for i in range(max_angle/step):
            self.move_relative(self._robot_name,
                               [0, 0, 0, 0, 0, radians(step)], self._speed)
            if self._take_sample:
                self._take_sample()

        self.move_relative(self._robot_name,
                           [0, 0, 0, 0, 0, radians(-max_angle)], self._speed)

        for i in range(max_angle/step):
            self.move_relative(self._robot_name,
                               [0, 0, 0, 0, 0, radians(-step)], self._speed)
            if self._take_sample:
                self._take_sample()

        self.move_relative(self._robot_name,
                           [0, 0, 0, 0, 0, radians(max_angle)], self._speed)

        if self._compute_calibration:
            res = self._compute_calibration()
            print('  compute calibration: {}').format(res.message)
            res = self._save_calibration()
            print('  save calibration: {}').format(res.message)

        self.go_to_named_pose(self._robot_name, 'home')


######################################################################
#  global functions                                                  #
######################################################################
if __name__ == '__main__':

    rospy.init_node('calib_ftsensor', anonymous=True)

    with FTCalibrationRoutines() as routines:
        routines.run()
