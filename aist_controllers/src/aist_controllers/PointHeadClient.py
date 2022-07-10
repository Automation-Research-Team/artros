# Copyright (C) 2020-2021, National Institute of Advanced Industrial Science
# and Technology (AIST), TOYOTA MOTOR CORPORATION, Ltd.
#
# Any using, copying, disclosing information regarding the software and
# documentation without permission of the copyright holders are prohibited.
# The software is provided "AS IS", without warranty of any kind, express or
# implied, including all implied warranties of merchantability and fitness.
# In no event shall the authors or copyright holders be liable for any claim,
# damages or other liability, whether in an action of contract, tort or
# otherwise, arising from, out of or in connection with the software or
# the use or other dealings in the software.
import rospy
from math                 import radians, degrees
from geometry_msgs.msg    import Point, Vector3
from actionlib            import SimpleActionClient
from aist_controllers.msg import PointHeadAction, PointHeadGoal

######################################################################
#  class PointHeadClient                                              #
######################################################################
class PointHeadClient(object):
    def __init__(self, server="/point_head_tracker"):
        super(PointHeadClient, self).__init__()

        self.target_point   = rospy.get_param('~target_point',  [0, 0, 0])
        self.pointing_axis  = rospy.get_param('~pointing_axis', [0, 0, 1])
        self.pointing_frame = rospy.get_param(
                                  '~pointing_frame',
                                  'biclops_camera_color_optical_frame')
        self.min_duration   = rospy.get_param('~min_duration', 0.05)
        self.max_velocity   = rospy.get_param('~max_velocity', 0.7)
        self._point_head    = SimpleActionClient(server + '/point_head',
                                                 PointHeadAction)

    @property
    def target_point(self):
        point = self._target_point
        return (point.x, point.y, point.z)

    @target_point.setter
    def target_point(self, target_point):
        self._trarget_point = Point(*target_point)

    @property
    def pointing_frame(self):
        return self._pointing_frame

    @pointing_frame.setter
    def pointing_frame(self, pointing_frame):
        self._pointing_frame = pointing_frame

    @property
    def min_duration(self):
        return self._min_duration.to_sec()

    @min_duration.setter
    def min_duration(self, min_duration):
        self._min_duration = ropy.Duration(min_duration)

    @property
    def max_velocity(self):
        return self._max_velocity

    @max_velocity.setter
    def max_velocity(self, max_velocity):
        self._max_velocity = max_velocity

    def send_goal(self, target_frame, feedback_cb=None):
        goal = amsg.PointHeadGoal()
        goal.target.header.stamp    = rospy.Time.now()
        goal.target.header.frame_id = target_frame
        goal.target.point           = self._target_point
        goal.pointing_frame         = self._pointing_frame
        goal.min_duration           = self._min_duration
        goal.max_velocity           = self._max_velocity

        self._point_head.send_goal(goal, feedback_cb=feedback_cb)

    def cancel_goal(self):
        self._point_head.cancel_goal()
