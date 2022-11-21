#!/usr/bin/env python3

import sys, rospy
from aist_camera_multiplexer import CameraMultiplexerClient

#########################################################################
#  main                                                                 #
#########################################################################
if __name__ == '__main__':

    camera_name = sys.argv[1]

    rospy.init_node('select_camera')
    multiplexer = CameraMultiplexerClient('camera_multiplexer')

    if not multiplexer.activate_camera(camera_name):
        print('Unknown camera[{}]'.format(camera_name))
