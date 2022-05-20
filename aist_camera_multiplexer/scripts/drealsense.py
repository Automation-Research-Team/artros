#!/usr/bin/env python3

import rospy
from aist_camera_multiplexer import RealSenseMultiplexerClient

#########################################################################
#  main                                                                 #
#########################################################################
if __name__ == '__main__':

    rospy.init_node('drealsense')

    multiplexer = RealSenseMultiplexerClient('camera_multiplexer')

    while not rospy.is_shutdown():
        for camera_name in multiplexer.camera_names:
            print(camera_name)
        try:
            camera_name = raw_input('Camera name >> ')
            if not multiplexer.activate_camera(camera_name):
                print('Unknown camera[{}]'.format(camera_name))
        except Exception as e:
            print e
