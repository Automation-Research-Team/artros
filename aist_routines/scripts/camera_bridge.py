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

import rospy, nep
from cv_bridge       import CvBridge
from sensor_msgs.msg import Image

#########################################################################
#  class CameraBridge                                                   #
#########################################################################
class CameraBridge(object):
    def __init__(self):
        super(CameraBridge, self).__init__()

        nep_node = nep.node('camera_web_py/script')
        nep_conf = nep_node(rospy.param('nep_ip', '163.220.51.108'))
        self._image_pub = nep_node.new_pub('camera_web/image', 'image',
                                           nep_conf)
        self._image_sub = rospy.Subscriber('/image', Image, self._image_cb,
                                           queue_size=1)
        rospy.loginfo('(CameraBridge) started')

    def _image_sub(image_msg):
        cvb = CvBridge()
        image = cvb.imgmsg_to_cv2(image_msg, 'bgr8')
        self._imge_pub.publish(image)


#########################################################################
#  entry point                                                          #
#########################################################################
if __name__ == '__main__':
    rospy.init_node('camera_bridge')

    camera_bridge = CameraBridge()
    rospy.spin()
