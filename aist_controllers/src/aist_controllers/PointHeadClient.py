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
from geometry_msgs.msg import Point, Vector3
from actionlib         import SimpleActionClient
from control_msgs.msg  import PointHeadAction, PointHeadGoal

######################################################################
#  class PointHeadClient                                              #
######################################################################
class PointHeadClient(object):
    def __init__(self, server='point_head_tracker'):
        super(PointHeadClient, self).__init__()

        self.pointing_axis  = rospy.get_param('~pointing_axis',
                                              [0, 0, 1])
        self.pointing_frame = rospy.get_param('~pointing_frame',
                                              'biclops_camera_color_optical_frame')
        self.min_duration   = rospy.get_param('~min_duration', 0.05)
        self.max_velocity   = rospy.get_param('~max_velocity', 0.7)
        self._point_head    = SimpleActionClient(server + '/point_head',
                                                 PointHeadAction)

        if self._point_head.wait_for_server(timeout=rospy.Duration(5)):
            rospy.loginfo('(PointHeadClient) connected to server[%s]',
                          server + '/point_head')
        else:
            self._point_head = None
            rospy.logerr('(PointHeadClient) failed to connect to server[%s]',
                         server + '/point_head')

    @property
    def is_connected(self):
        return self._point_head is not None

    @property
    def pointing_axis(self):
        axis = self._pointing_axis
        return (axis.x, axis.y, axis.z)

    @pointing_axis.setter
    def pointing_axis(self, pointing_axis):
        self._pointing_axis = Vector3(*pointing_axis)

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
        self._min_duration = rospy.Duration(min_duration)

    @property
    def max_velocity(self):
        return self._max_velocity

    @max_velocity.setter
    def max_velocity(self, max_velocity):
        self._max_velocity = max_velocity

    def send_goal(self, target_frame, target_point=[0, 0, 0],
                  feedback_cb=None):
        goal = PointHeadGoal()
        goal.target.header.stamp    = rospy.Time.now()
        goal.target.header.frame_id = target_frame
        goal.target.point           = Point(*target_point)
        goal.pointing_axis          = self._pointing_axis
        goal.pointing_frame         = self._pointing_frame
        goal.min_duration           = self._min_duration
        goal.max_velocity           = self._max_velocity

        self._point_head.send_goal(goal, feedback_cb=feedback_cb)

    def cancel_goal(self):
        self._point_head.cancel_goal()

    def get_state(self):
        return self._point_head.get_state()
