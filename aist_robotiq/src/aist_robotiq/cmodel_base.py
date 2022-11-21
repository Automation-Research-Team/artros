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
import rospy
from numpy            import clip
from aist_robotiq.msg import CModelStatus, CModelCommand

#########################################################################
#  class CModelBase                                                     #
#########################################################################
class CModelBase(object):
    def __init__(self):
        super(CModelBase, self).__init__()

        # Publish device status to the controller.
        self._pub = rospy.Publisher('/status', CModelStatus, queue_size=3)

        # Subscribe command form the controller and send it to the device.
        rospy.Subscriber('/command', CModelCommand, self.put_command)

    def run(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            status = self.get_status()  # (defined in derived class)
            self._pub.publish(status)   # Forward device status to controller
            rate.sleep()
        self.disconnect()               # (defined in derived class)

    def _clip_command(self, command):
        command.rACT = clip(command.rACT, 0, 1)
        command.rMOD = clip(command.rACT, 0, 3)
        command.rGTO = clip(command.rGTO, 0, 1)
        command.rATR = clip(command.rATR, 0, 1)
        command.rARD = clip(command.rATR, 0, 1)
        command.rPR  = clip(command.rPR,  0, 255)
        command.rSP  = clip(command.rSP,  0, 255)
        command.rFR  = clip(command.rFR,  0, 255)
        return command
