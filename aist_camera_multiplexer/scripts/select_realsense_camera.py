#!/usr/bin/env python3

import sys, rospy
from aist_camera_multiplexer import RealSenseMultiplexerClient

#########################################################################
#  main                                                                 #
#########################################################################
if __name__ == '__main__':

    camera_name = sys.argv[1]

    rospy.init_node('select_realsense_camera')
    multiplexer = RealSenseMultiplexerClient('camera_multiplexer')

    if not multiplexer.activate_camera(camera_name):
        print('Unknown camera[{}]'.format(camera_name))
