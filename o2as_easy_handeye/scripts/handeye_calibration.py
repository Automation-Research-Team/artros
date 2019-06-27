#!/usr/bin/env python

import sys
import os
import rospy
import argparse
from math import radians, degrees
from std_srvs.srv  import Empty, Trigger
from geometry_msgs import msg as gmsg
from tf import TransformListener, transformations as tfs

from o2as_easy_handeye.srv import GetSampleList, ComputeCalibration

from o2as_routines.base import O2ASBaseRoutines
from aist_routines.base import AISTBaseRoutines

# http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython
import moveit_commander
import sensor_msgs.msg
import cv2
from cv_bridge import CvBridge, CvBridgeError

initposes = {
    'o2as': {
        'a_phoxi_m_camera': {
            'a_bot': [
                0.10, -0.10, 0.35, radians(  0), radians( 90), radians(0)],
            'b_bot': [
                0.10,  0.10, 0.35, radians(  0), radians( 90), radians(0)],
            'c_bot': [
                -0.30, 0.00, 0.35, radians(  0), radians( 90), radians(0)],
        },
        'a_bot_camera': {
            'a_bot': [
                0.00, -0.20, 0.35, radians(-90), radians( 90), radians(0)],
        },
    },

    'aist': {
        'a_phoxi_m_camera': {
            'a_bot': [
                0.10, -0.20, 0.35, radians(  0), radians( 90), radians(0)],
            'b_bot': [
                0.10,  0.10, 0.35, radians(  0), radians( 90), radians(0)],
        },
    },

    'pgrp': {
        'a_phoxi_m_camera': {
            'a_bot': [
                0.10, -0.20, 0.35, radians(  0), radians( 90), radians(0)],
            'b_bot': [
                0.10,  0.10, 0.35, radians(  0), radians( 90), radians(0)],
        },
        'a_bot_camera': {
            'a_bot': [
                0.00, -0.20, 0.35, radians(  0), radians( 90), radians(0)],
        },
    },

    'ur5e': {
        'a_phoxi_m_camera': {
            'c_bot': [
                0.10, -0.20, 0.35, radians(  0), radians( 90), radians(0)],
            'd_bot': [
                0.10,  0.10, 0.35, radians(  0), radians( 90), radians(0)],
        },
    },
}

# Poses taken during handeye calibration
keyposes = {
    'o2as': {
        'a_phoxi_m_camera': {
            'a_bot': [
                [0.15, -0.20, 0.25, radians( 30), radians( 25), radians(0)],
                [0.15, -0.10, 0.25, radians( 30), radians( 25), radians(0)],
                [0.15,  0.00, 0.25, radians( 30), radians( 25), radians(0)],

                [0.15,  0.00, 0.35, radians( 30), radians( 25), radians(0)],
                [0.15, -0.10, 0.35, radians( 30), radians( 25), radians(0)],
                [0.15, -0.20, 0.35, radians( 30), radians( 25), radians(0)],
            ],

            'b_bot': [
                # configulation for real
                [0.15,  0.15, 0.18, radians( 30), radians( 25), radians(0)],
                [0.15,  0.00, 0.18, radians( 30), radians( 25), radians(0)],
                [0.15, -0.15, 0.18, radians( 30), radians( 25), radians(0)],

                [0.15, -0.10, 0.28, radians( 30), radians( 25), radians(0)],
                [0.15,  0.00, 0.28, radians( 30), radians( 25), radians(0)],
                [0.15,  0.10, 0.28, radians( 30), radians( 25), radians(0)],

                # [0.20,  0.15, 0.20, radians( 30), radians( 25), radians(0)],
                # [0.20,  0.00, 0.20, radians( 30), radians( 25), radians(0)],
                # [0.20, -0.15, 0.20, radians(  0), radians( 25), radians(0)],

                # [0.15, -0.10, 0.10, radians(  0), radians( 25), radians(0)],
                # [0.15,  0.05, 0.10, radians( 30), radians( 25), radians(0)],
                # [0.15,  0.20, 0.10, radians( 30), radians( 25), radians(0)],
            ],

            'c_bot': [
                [0.03, -0.10, 0.19, radians( 30), radians( 25), radians(0)],
                [0.03,  0.00, 0.19, radians( 30), radians( 25), radians(0)],
                [0.03,  0.10, 0.19, radians( 30), radians( 25), radians(0)],
            ]
        },

        'a_bot_camera': {
            'a_bot': [
                [-0.05, -0.20, 0.20, radians(  0), radians( 60), radians(  0)],
                [-0.05, -0.17, 0.20, radians(  0), radians( 75), radians(  0)],
                [-0.05, -0.15, 0.20, radians(  0), radians( 90), radians(  0)],
                [-0.05, -0.10, 0.20, radians(  0), radians(100), radians(  0)],
                [-0.05, -0.05, 0.20, radians(  0), radians(110), radians(  0)],
                [-0.05, -0.05, 0.20, radians(-90), radians( 70), radians(-90)],
                [-0.05, -0.12, 0.20, radians(-90), radians( 85), radians(-90)],
                [-0.05, -0.14, 0.20, radians(-90), radians(105), radians(-90)],
                [-0.05, -0.16, 0.20, radians(-90), radians(120), radians(-90)],
            ]
        },
    },

    'aist': {
        'a_phoxi_m_camera': {
            'a_bot': [
                [0.05, -0.10, 0.16, radians( 30), radians( 25), radians(0)],
                [0.05,  0.00, 0.16, radians( 30), radians( 25), radians(0)],
                [0.05,  0.10, 0.16, radians( 30), radians( 25), radians(0)],

                [0.05,  0.10, 0.25, radians( 30), radians( 25), radians(0)],
                [0.05,  0.00, 0.25, radians( 30), radians( 25), radians(0)],
                [0.05, -0.10, 0.25, radians( 30), radians( 25), radians(0)],
            ],

            'b_bot': [
                [0.15,  0.20, 0.16, radians( 30), radians( 25), radians(0)],
                [0.15,  0.10, 0.16, radians( 30), radians( 25), radians(0)],
                [0.15,  0.00, 0.16, radians( 30), radians( 25), radians(0)],

                [0.15,  0.00, 0.25, radians( 30), radians( 25), radians(0)],
                [0.15,  0.10, 0.25, radians( 30), radians( 25), radians(0)],
                [0.15,  0.20, 0.25, radians( 30), radians( 25), radians(0)],

                # [0.40,  0.15, 0.15, radians( 30), radians( 25), radians(0)],
                # [0.40,  0.00, 0.15, radians( 30), radians( 25), radians(0)],
                # [0.40, -0.15, 0.15, radians(  0), radians( 25), radians(0)],

                # [0.35, -0.10, 0.10, radians(  0), radians( 25), radians(0)],
                # [0.35,  0.05, 0.10, radians( 30), radians( 25), radians(0)],
                # [0.35,  0.20, 0.10, radians( 30), radians( 25), radians(0)],
            ],
        },
    },

    'pgrp': {
        'a_phoxi_m_camera': {
            'a_bot': [
                [0.05, -0.10, 0.14, radians( 30), radians( 25), radians(0)],
                [0.05,  0.00, 0.14, radians( 30), radians( 25), radians(0)],
                [0.05,  0.10, 0.14, radians( 30), radians( 25), radians(0)],

                [0.05,  0.10, 0.25, radians( 30), radians( 25), radians(0)],
                [0.05,  0.00, 0.25, radians( 30), radians( 25), radians(0)],
                [0.05, -0.10, 0.25, radians( 30), radians( 25), radians(0)],
            ],

            'b_bot': [
                [0.20,  0.10, 0.16, radians( 30), radians( 25), radians(0)],
                [0.20,  0.00, 0.16, radians( 30), radians( 25), radians(0)],
                [0.20, -0.15, 0.16, radians(  0), radians( 25), radians(0)],

                # [0.15, -0.15, 0.25, radians( 30), radians( 25), radians(0)],
                # [0.15,  0.00, 0.25, radians( 30), radians( 25), radians(0)],
                # [0.15,  0.15, 0.25, radians( 30), radians( 25), radians(0)],

                # [0.40,  0.15, 0.15, radians( 30), radians( 25), radians(0)],
                # [0.40,  0.00, 0.15, radians( 30), radians( 25), radians(0)],
                # [0.40, -0.15, 0.15, radians(  0), radians( 25), radians(0)],

                [-0.00, -0.15, 0.16, radians(  0), radians( 25), radians(0)],
                [-0.00,  0.00, 0.16, radians( 30), radians( 25), radians(0)],
                [-0.00,  0.15, 0.16, radians( 30), radians( 25), radians(0)],
            ],
        },

        'a_bot_camera': {
            'a_bot': [
                [-0.10, -0.15, 0.35, radians(  0), radians( 60), radians(  0)],
                [-0.07, -0.17, 0.35, radians(  0), radians( 75), radians(  0)],
                [-0.05, -0.15, 0.35, radians(  0), radians( 90), radians(  0)],
                [-0.05, -0.13, 0.35, radians(  0), radians(100), radians(  0)],
                [-0.05, -0.10, 0.35, radians(  0), radians(110), radians(  0)],
                [-0.05, -0.03, 0.35, radians(-90), radians( 70), radians(-90)],
                [-0.05, -0.10, 0.35, radians(-90), radians( 85), radians(-90)],
                [-0.05, -0.20, 0.35, radians(-90), radians(105), radians(-90)],
                [-0.05, -0.30, 0.35, radians(-90), radians(120), radians(-90)],
            ]
        },
    },

    'ur5e': {
        'a_phoxi_m_camera': {
            'c_bot': [
                [0.15, -0.15, 0.16, radians( 30), radians( 25), radians(0)],
                [0.15, -0.05, 0.16, radians( 30), radians( 25), radians(0)],
                [0.15,  0.05, 0.16, radians( 30), radians( 25), radians(0)],

                [0.15,  0.05, 0.25, radians( 30), radians( 25), radians(0)],
                [0.15, -0.00, 0.25, radians( 30), radians( 25), radians(0)],
                [0.15, -0.15, 0.25, radians( 30), radians( 25), radians(0)],
            ],

            'd_bot': [
                [0.15,  0.20, 0.16, radians( 30), radians( 25), radians(0)],
                [0.15,  0.10, 0.16, radians( 30), radians( 25), radians(0)],
                [0.15,  0.00, 0.16, radians( 30), radians( 25), radians(0)],

                [0.15,  0.00, 0.25, radians( 30), radians( 25), radians(0)],
                [0.15,  0.10, 0.25, radians( 30), radians( 25), radians(0)],
                [0.15,  0.20, 0.25, radians( 30), radians( 25), radians(0)],
            ],
        },

        'a_bot_camera': {
            'a_bot': [
                [-0.05, -0.20, 0.20, radians(  0), radians( 60), radians(  0)],
                [-0.05, -0.17, 0.20, radians(  0), radians( 75), radians(  0)],
                [-0.05, -0.15, 0.20, radians(  0), radians( 90), radians(  0)],
                [-0.05, -0.10, 0.20, radians(  0), radians(100), radians(  0)],
                [-0.05, -0.05, 0.20, radians(  0), radians(110), radians(  0)],
                [-0.05, -0.05, 0.20, radians(-90), radians( 70), radians(-90)],
                [-0.05, -0.12, 0.20, radians(-90), radians( 85), radians(-90)],
                [-0.05, -0.14, 0.20, radians(-90), radians(105), radians(-90)],
                [-0.05, -0.16, 0.20, radians(-90), radians(120), radians(-90)],
            ]
        },
    },
}


######################################################################
#  class HandEyeCalibrationRoutines                                  #
######################################################################
class HandEyeCalibrationRoutines(AISTBaseRoutines):
    def __init__(self, camera_name, robot_name, speed, sleep_time, needs_calib):
        super(HandEyeCalibrationRoutines, self).__init__()

        self._robot_name  = robot_name
        self._camera_name = camera_name
        self._speed       = speed
        self._sleep_time  = sleep_time

        if needs_calib:
            ns = "/{}_from_{}/".format(camera_name, robot_name)
            self.get_sample_list = rospy.ServiceProxy(ns + "get_sample_list",
                                                       GetSampleList)
            self.take_sample = rospy.ServiceProxy(ns + "take_sample", Trigger)
            self.compute_calibration = rospy.ServiceProxy(
                ns + "compute_calibration", ComputeCalibration)
            self.save_calibration = rospy.ServiceProxy(ns + "save_calibration",
                                                       Trigger)
            self.reset = rospy.ServiceProxy(ns + "reset", Empty)
        else:
            self.get_sample_list     = None
            self.take_sample         = None
            self.compute_calibration = None
            self.save_calibration    = None
            self.reset               = None

    def is_eye_on_hand(self):
        return self._camera_name == self._robot_name + "_camera"

    def go_home(self):
        self.go_to_named_pose("home", self._robot_name)

    def save_image(self, file_name):
        img_msg = rospy.wait_for_message("/aruco_tracker/result",
                                         sensor_msgs.msg.Image,
                                         timeout=10.0)
        bridge = CvBridge()
        cv2.imwrite(file_name, bridge.imgmsg_to_cv2(img_msg, "bgr8"))

    def move(self, pose):
        poseStamped = gmsg.PoseStamped()
        poseStamped.header.frame_id = "workspace_center"
        poseStamped.pose = gmsg.Pose(
            gmsg.Point(pose[0], pose[1], pose[2]),
            gmsg.Quaternion(
                *tfs.quaternion_from_euler(pose[3], pose[4], pose[5])))
        print("  move to " + self.format_pose(poseStamped))
        (success, _, current_pose) \
            = self.go_to_pose_goal(
                self._robot_name, poseStamped, self._speed,
                end_effector_link=self._robot_name + "_ee_link",
                move_lin=True)
        print("  reached " + self.format_pose(current_pose))
        return success

    def move_to(self, pose, keypose_num, subpose_num):
        if not self.move(pose):
            return False

        rospy.sleep(self._sleep_time)  # Wait until the robot settles.

        self.start_acquisition(self._camera_name)

        rospy.sleep(self._sleep_time)

        # try:
        #     self.save_image("aruco_result-{:0=2}-{:0=2}.jpeg".format(
        #         keypose_num, subpose_num))
        # except CvBridgeError, e:
        #     print(e)
        # except rospy.ROSException, e:
        #     print(e)

        success = True
        if self.take_sample:
            try:
                res = self.take_sample()
                n = len(self.get_sample_list().cMo)
                print("  {} samples taken: {}").format(n, res.message)
            except rospy.ServiceException as e:
                print "Service call failed: %s" % e
                success = False

        self.stop_acquisition(self._camera_name)

        return success

    def move_to_subposes(self, pose, keypose_num):
        roll = pose[3]
        for i in range(3):
            print("\n--- Subpose [{}/5]: Try! ---".format(i + 1))
            if self.move_to(pose, keypose_num, i + 1):
                print("--- Subpose [{}/5]: Succeeded. ---".format(i + 1))
            else:
                print("--- Subpose [{}/5]: Failed. ---".format(i + 1))
            pose[3] -= radians(30)

        pose[3] = roll - radians(30)
        pose[4] += radians(15)

        for i in range(2):
            print("\n--- Subpose [{}/5]: Try! ---".format(i + 4))
            if self.move_to(pose, keypose_num, i + 4):
                print("--- Subpose [{}/5]: Succeeded. ---".format(i + 4))
            else:
                print("--- Subpose [{}/5]: Failed. ---".format(i + 4))
            pose[4] -= radians(30)

    def run(self, initpose, keyposes):
        self.stop_acquisition(self._camera_name)

        if self.reset:
            self.reset()

        # Reset pose
        self.go_home()
        self.move(initpose)

        # Collect samples over pre-defined poses
        for i, keypose in enumerate(keyposes, 1):
            print("\n*** Keypose [{}/{}]: Try! ***".format(i, len(keyposes)))
            if self.is_eye_on_hand():
                self.move_to(keypose, i, 1)
            else:
                self.move_to_subposes(keypose, i)
            print("*** Keypose [{}/{}]: Completed. ***".format(i,
                                                               len(keyposes)))

        if self.compute_calibration:
            res = self.compute_calibration()
            print(res.message)
            res = self.save_calibration()
            print(res.message)

        self.go_home()


######################################################################
#  global functions                                                  #
######################################################################
if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Do hand-eye calibration')
    parser.add_argument('-C',
                        '--config',
                        action='store',
                        nargs='?',
                        default='aist',
                        type=str,
                        choices=None,
                        help='configuration name',
                        metavar=None)
    parser.add_argument('-c',
                        '--camera_name',
                        action='store',
                        nargs='?',
                        default='a_phoxi_m_camera',
                        type=str,
                        choices=None,
                        help='camera name',
                        metavar=None)
    parser.add_argument('-r',
                        '--robot_name',
                        action='store',
                        nargs='?',
                        default='b_bot',
                        type=str,
                        choices=None,
                        help='robot name',
                        metavar=None)
    parser.add_argument('-v',
                        '--visit',
                        action='store_true',
                        help='only visit calibration points')

    args = parser.parse_args()

    assert (args.camera_name in {"a_phoxi_m_camera", "a_bot_camera"})
    assert (args.robot_name  in {"a_bot", "b_bot", "c_bot", "d_bot"})

    speed = 1
    sleep_time = 1
    with HandEyeCalibrationRoutines(args.camera_name, args.robot_name,
                                              speed, sleep_time,
                                              not args.visit) as routines:

        print("=== Calibration started for {} + {} ===".format(
            args.camera_name, args.robot_name))
        routines.run(initposes[args.config][args.camera_name][args.robot_name],
                     keyposes[ args.config][args.camera_name][args.robot_name])
        print("=== Calibration completed for {} + {} ===".format(
            args.camera_name, args.robot_name))
