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
import os
import rospy

from tf import TransformBroadcaster, TransformListener, transformations as tfs
from geometry_msgs import msg as gmsg
from math import degrees

#########################################################################
#  local functions                                                      #
#########################################################################
class CalibrationPublisher(object):
    def __init__(self):
        super(CalibrationPublisher, self).__init__()

        self._broadcaster = TransformBroadcaster()
        self._listener    = TransformListener()

        self._transform = gmsg.TransformStamped()
        self._transform.header.frame_id = rospy.get_param("~parent")
        self._transform.child_frame_id  = rospy.get_param("~child")
        T = rospy.get_param("~transform")
        self._transform.transform \
            = gmsg.Transform(gmsg.Vector3(T["x"], T["y"], T["z"]),
                             gmsg.Quaternion(T["qx"], T["qy"],
                                             T["qz"], T["qw"]))

    def __enter__(self):
        return self

    def __exit__(self, exception_type, exception_value, traceback):
        # Get camera(tcp) <- camera(body) transform
        tcp_body = self._get_transform(
                     rospy.get_param("~tcp_frame"),
                     rospy.get_param("~body_frame"))
        bot_tcp  = ((self._transform.transform.translation.x,
                     self._transform.transform.translation.y,
                     self._transform.transform.translation.z),
                    (self._transform.transform.rotation.x,
                     self._transform.transform.rotation.y,
                     self._transform.transform.rotation.z,
                     self._transform.transform.rotation.w))

        mat = tfs.concatenate_matrices(
                self._listener.fromTranslationRotation(*bot_tcp),
                self._listener.fromTranslationRotation(*tcp_body))
        print("\n=== Estimated effector/base <- camera_body transform ===")
        self._print_mat(mat)
        print("\n")
        return True

    def run(self):
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            self._transform.header.stamp = rospy.Time.now()
            self._broadcaster.sendTransformMessage(self._transform)
            rate.sleep()

    def _get_transform(self, target_frame, source_frame):
        now = rospy.Time.now()
        self._listener.waitForTransform(target_frame, source_frame, now,
                                        rospy.Duration(10))
        return self._listener.lookupTransform(target_frame, source_frame, now)

    def _print_mat(self, mat):
        xyz = tfs.translation_from_matrix(mat)
        rpy = map(degrees, tfs.euler_from_matrix(mat))
        print('<origin xyz="{0[0]} {0[1]} {0[2]}" rpy="${{{1[0]}*pi/180}} ${{{1[1]}*pi/180}} ${{{1[2]}*pi/180}}"/>'.format(xyz, rpy))

    def _print_transform(self, transform):
        xyz = (transform.translation.x,
               transform.translation.y, transform.translation.z)
        rpy = map(degrees, tfs.euler_from_quaternion((transform.rotation.x,
                                                      transform.rotation.y,
                                                      transform.rotation.z,
                                                      transform.rotation.w)))
        print('<origin xyz="{0[0]} {0[1]} {0[2]}" rpy="${{{1[0]}*pi/180}} ${{{1[1]}*pi/180}} ${{{1[2]}*pi/180}}"/>'.format(xyz, rpy))


#########################################################################
#  main part                                                            #
#########################################################################
if __name__ == "__main__":
    rospy.init_node("publish_calibration")

    while rospy.get_time() == 0.0:
        pass

    with CalibrationPublisher() as cp:
        cp.run()
