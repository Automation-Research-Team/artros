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
import rospy, threading
from aist_controllers.PointHeadClient import PointHeadClient
from aist_utility.compat              import *

######################################################################
#  class InteractivePointHeadClient                                  #
######################################################################
class InteractivePointHeadClient(PointHeadClient):
    def __init__(self, server):
        super(InteractivePointHeadClient, self).__init__(server)

        self._target_frame = rospy.get_param('~target_frame', 'marker_frame')
        self._target_point = rospy.get_param('~target_point', [0, 0, 0])
        thread = threading.Thread(target=self._interactive)
        thread.start()

    def _interactive(self):
        while not rospy.is_shutdown():
            try:
                key = raw_input('>> ')
                if key == 'q':
                    break
                elif key == 'c':
                    self.cancel_goal()
                else:
                    self.send_goal(self._target_frame, self._target_point)
            except Exception as e:
                print(e.message)


if __name__ == '__main__':

    rospy.init_node('point_head_client', anonymous=True)

    server = rospy.get_param('~server', 'point_head_tracker')
    point_head_client = InteractivePointHeadClient(server)
    rospy.spin()
