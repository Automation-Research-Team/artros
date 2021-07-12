#!/usr/bin/env python

import sys, rospy
from aist_camera_multiplexer import RealSenseMultiplexerClient

#########################################################################
#  main                                                                 #
#########################################################################
if __name__ == '__main__':

    camera_name = sys.argv[1]

    rospy.init_node('select_realsesne_camera')
    multiplexer = RealSenseMultiplexerClient('camera_multiplexer')
    multiplexer.activate_camera(camera_name)
