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
import rospy, collections
from aist_routines import AISTBaseRoutines
from aist_routines import msg as amsg

if __name__ == '__main__':

    rospy.init_node("test", anonymous=True)

    robots   = rospy.get_param("~robots")
    cameras  = rospy.get_param("~cameras")
    grippers = {}

    for robot_name, robot in robots.items():
        if "grippers" in robot:
            grippers.update(robot["grippers"])
        if "cameras" in robot:
            cameras.update(robot["cameras"])

    # for camera_name, camera in cameras.items():
    #     print camera_name, ':', camera

    # for gripper_name, gripper in grippers.items():
    #     print gripper_name, ':', gripper

    gripper_list = {}
    camera_list = {}
    for robot_name, robot in robots.items():
        if "grippers" in robot:
            for gripper in robot["grippers"].values():
                gripper_list[robot_name] = gripper
        if "cameras" in robot:
            for camera in robot["cameras"].values():
                camera_list[robot_name] = camera

    print gripper_list
    print camera_list
