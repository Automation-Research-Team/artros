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

import copy
import threading
import rospy
import actionlib
import geometry_msgs.msg as gmsg
import tf.transformations as tfs

from math                import pi, radians, degrees
from move_base_msgs.msg  import MoveBaseAction, MoveBaseGoal
from nav_msgs            import msg as nmsg
from geometry_msgs       import msg as gmsg
from tf                  import TransformListener, transformations as tfs

######################################################################
#  class MoveBaseClient                                              #
######################################################################
class MoveBaseClient(object):
    def __init__(self):
        super(MoveBaseClient, self).__init__()

        self._move_base = actionlib.SimpleActionClient("/move_base",
                                                       MoveBaseAction)
        if self._move_base.wait_for_server(rospy.Duration(10)):
            rospy.loginfo("Connected to move_base.")
        else:
            rospy.logerr("MoveBaseClient.__init()__: failed to connect move_base action server")

        self._odom_sub          = rospy.Subscriber("odom", nmsg.Odometry,
                                                   self._odom_callback)
        self._odom_recv_event   = threading.Event()
        self._current_odom      = nmsg.Odometry()
        self._current_odom.header.stamp = rospy.Time.now()
        self._current_odom.header.frame_id = "odom"
        self._reference_frame   = "map"
        self._listener          = TransformListener()

    @property
    def current_odom(self):
        return self._current_odom

    def move_base(self, x, y, theta, frame="map", wait=True):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id  = frame
        goal.target_pose.pose.position    = gmsg.Vector3(x, y, 0)
        goal.target_pose.pose.orientation = gmsg.Quaternion(
            *tfs.quaternion_from_euler(0, 0, theta))
        return self._move_base.send_goal_and_wait(goal)

    def move_base_to_frame(self, target_frame):
        return self.move_base(0, 0, 0, target_frame)

    def format_odom(self, odom):
        xyzrpy = self._xyz_rpy(odom)
        return "[{:.4f}, {:.4f} ; {:.2f}]".format(xyzrpy[0], xyzrpy[1],
                                                  degrees(xyzrpy[5]))

    def _xyz_rpy(self, odom):
        try:
            pose = gmsg.PoseStamped()
            pose.header = odom.header
            pose.pose   = odom.pose.pose
            self._listener.waitForTransform(self._reference_frame,
                                            pose.header.frame_id,
                                            pose.header.stamp,
                                            rospy.Duration(10))
            transformed_pose = self._listener.transformPose(
                                self._reference_frame, pose).pose
        except Exception as e:
            rospy.logerr("MoveBaseClient._xyz_rpy(): {}".format(e))
            raise e

        rpy = tfs.euler_from_quaternion([transformed_pose.orientation.x,
                                         transformed_pose.orientation.y,
                                         transformed_pose.orientation.z,
                                         transformed_pose.orientation.w])
        return [transformed_pose.position.x,
                transformed_pose.position.y,
                transformed_pose.position.z,
                rpy[0], rpy[1], rpy[2]]

    def _odom_callback(self, odom):
        self._odom_recv_event.clear()
        self._current_odom = copy.deepcopy(odom)
        self._odom_recv_event.set()
