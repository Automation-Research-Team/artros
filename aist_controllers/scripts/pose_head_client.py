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
from math                            import radians
from tf                              import transformations as tfs
from geometry_msgs.msg               import (PoseStamped,
                                             Pose, Point, Quaternion)
from aist_controllers.PoseHeadClient import PoseHeadClient
from controller_manager_msgs.srv     import SwitchController

######################################################################
#  class InteractivePoseHeadClient                                   #
######################################################################
class InteractivePoseHeadClient(PoseHeadClient):
    def __init__(self, server):
        super(InteractivePoseHeadClient, self).__init__(server)

        xyzrpy = rospy.get_param('~target_pose', [0, 0, 0.3, 0, 90, 0])
        target_pose = PoseStamped()
        target_pose.header.frame_id = rospy.get_param('~target_frame',
                                                      'marker_frame')
        target_pose.pose = Pose(Point(*xyzrpy[0:3]),
                                Quaternion(*tfs.quaternion_from_euler(
                                    *map(radians, xyzrpy[3:6]))))
        self._target_pose = target_pose
        self._switch_client \
            = rospy.ServiceProxy('/controller_manager/switch_controller',
                                 SwitchController)

        thread = threading.Thread(target=self._interactive)
        thread.start()

    def _interactive(self):
        while not rospy.is_shutdown():
            try:
                key = raw_input('>> ')
                if key == 'q':
                    self.cancel_goal()
                    break
                elif key == 'c':
                    self.cancel_goal()
                else:
                    self.send_goal(self._target_pose)
            except Exception as e:
                print(e.message)


if __name__ == '__main__':
    rospy.init_node('pose_head_client', anonymous=True)

    server = rospy.get_param('~server', 'pose_head_tracker')
    pose_head_client = InteractivePoseHeadClient(server)
    rospy.spin()
