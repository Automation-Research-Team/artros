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
import copy
import collections

from math import pi
from tf   import transformations as tfs

from geometry_msgs.msg      import Vector3, Pose, Point, Quaternion
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg           import ColorRGBA

######################################################################
#  class MarkerPublisher                                             #
######################################################################
class MarkerPublisher(object):
    MarkerProps = collections.namedtuple("MarkerProps",
                                         "draw_axes, scale, color")
    _marker_props = {
        "pose" :
            MarkerProps(True,  (0.004, 0.010, 0.010), (0.0, 1.0, 0.0, 0.8)),
        "pick_pose":
            MarkerProps(True,  (0.004, 0.010, 0.010), (1.0, 0.0, 1.0, 0.8)),
        "place_pose":
            MarkerProps(True,  (0.004, 0.010, 0.010), (0.0, 1.0, 1.0, 0.8)),
        "graspability":
            MarkerProps(True,  (0.004, 0.004, 0.004), (1.0, 1.0, 0.0, 0.8)),
        "":
            MarkerProps(False, (0.004, 0.004, 0.004), (0.0, 1.0, 0.0, 0.8)),
        }

    def __init__(self):
        super(MarkerPublisher, self).__init__()
        self._pub = rospy.Publisher("visualization_marker",
                                    MarkerArray, queue_size=10)
        self._markers = MarkerArray()

    def delete_all(self):
        marker        = Marker()
        marker.action = Marker.DELETEALL
        marker.ns     = "markers"
        self._markers.markers = [marker]
        self.publish()
        del self._markers.markers[:]

    def add(self, marker_type, marker_pose, endpoint=None,
            text="", lifetime=15):
        marker_prop = MarkerPublisher._marker_props[marker_type]

        marker              = Marker()
        marker.header       = marker_pose.header
        marker.header.stamp = rospy.Time.now()
        marker.ns           = "markers"
        marker.action       = Marker.ADD
        marker.lifetime     = rospy.Duration(lifetime)

        if marker_prop.draw_axes:  # Draw frame axes?
            smax = max(*marker_prop.scale)
            smin = min(*marker_prop.scale)

            marker.type  = Marker.ARROW
            marker.pose  = marker_pose.pose
            marker.scale = Vector3(2.5*smax, 0.5*smin, 0.5*smin)
            marker.color = ColorRGBA(1.0, 0.0, 0.0, 0.8)         # red
            marker.id    = len(self._markers.markers)
            self._markers.markers.append(copy.deepcopy(marker))  # x-axis

            marker.pose  = self._pose_rotated_by_rpy(marker_pose.pose,
                                                     0, 0, pi/2)
            marker.color = ColorRGBA(0.0, 1.0, 0.0, 0.8)         # green
            marker.id    = len(self._markers.markers)
            self._markers.markers.append(copy.deepcopy(marker))  # y-axis

            marker.pose  = self._pose_rotated_by_rpy(marker_pose.pose,
                                                     0, -pi/2, 0)
            marker.color = ColorRGBA(0.0, 0.0, 1.0, 0.8)         # blue
            marker.id    = len(self._markers.markers)
            self._markers.markers.append(copy.deepcopy(marker))  # z-axis

        if endpoint is None:
            marker.type  = Marker.SPHERE
            marker.pose  = marker_pose.pose
            marker.scale = Vector3(*marker_prop.scale)
            marker.color = ColorRGBA(*marker_prop.color)
            marker.id    = len(self._markers.markers)
            self._markers.markers.append(copy.deepcopy(marker))
        else:
            marker.type    = Marker.LINE_LIST
            marker.pose    = Pose(Point(0, 0, 0), Quaternion(0, 0, 0, 1))
            marker.points  = [marker_pose.pose.position, endpoint]
            marker.scale.x = 0.1*min(*marker_prop.scale)
            marker.scale.y = 0
            marker.scale.z = 0
            marker.color   = ColorRGBA(*marker_prop.color)
            marker.id      = len(self._markers.markers)
            self._markers.markers.append(copy.deepcopy(marker))

            marker.type  = Marker.SPHERE
            marker.pose  = Pose(endpoint, marker_pose.pose.orientation)
            del marker.points[:]
            marker.scale = Vector3(*marker_prop.scale)
            marker.color = ColorRGBA(*marker_prop.color)
            marker.id    = len(self._markers.markers)
            self._markers.markers.append(copy.deepcopy(marker))

        if text != "":
            marker.scale.x = 0
            marker.scale.y = 0
            marker.scale.z = min(*marker_prop.scale)
            marker.pose.position.z -= (marker.scale.z + 0.001)
            marker.type  = Marker.TEXT_VIEW_FACING
            marker.color = ColorRGBA(1.0, 1.0, 1.0, 0.8)  # white
            marker.text  = text
            marker.id    = len(self._markers.markers)
            self._markers.markers.append(copy.deepcopy(marker))

    def publish(self):
        self._pub.publish(self._markers)

    def _pose_rotated_by_rpy(self, pose, roll, pitch, yaw):
        pose_rotated = copy.deepcopy(pose)
        pose_rotated.orientation = Quaternion(
            *tfs.quaternion_multiply((pose.orientation.x, pose.orientation.y,
                                      pose.orientation.z, pose.orientation.w),
                                     tfs.quaternion_from_euler(roll,
                                                               pitch, yaw)))
        return pose_rotated
