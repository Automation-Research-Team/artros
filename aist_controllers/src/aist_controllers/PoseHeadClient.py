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
from tf                   import transformations as tfs
from geometry_msgs.msg    import Pose, Point, Quaternion
from actionlib            import SimpleActionClient
from aist_controllers.msg import PoseHeadAction, PoseHeadGoal

######################################################################
#  class PoseHeadClient                                              #
######################################################################
class PoseHeadClient(object):
    def __init__(self, server="pose_head_tracker"):
        super(PoseHeadClient, self).__init__()

        ns = '~' + server.strip('/')

        self.pointing_frame = rospy.get_param(ns + '/pointing_frame',
                                              'a_bot_outside_camera_color_optical_frame')
        self.min_duration   = rospy.get_param(ns + '/min_duration', 0.05)
        self.max_velocity   = rospy.get_param(ns + '/max_velocity', 0.7)
        self._pose_head     = SimpleActionClient(server + '/pose_head',
                                                 PoseHeadAction)

        if self._pose_head.wait_for_server(timeout=rospy.Duration(5)):
            rospy.loginfo('(PoseHeadClient) connected to server[%s]',
                          server + '/pose_head')
        else:
            self._pose_head = None
            rospy.logerr('(PoseHeadClient) failed to connect to server[%s]',
                         server + '/pose_head')


    @property
    def is_connected(self):
        return self._pose_head is not None

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

    def send_goal(self, target_frame, target_pose=[0, 0, 0.3, 180, 0, 0],
                  feedback_cb=None):
        goal = PoseHeadGoal()
        goal.target.header.stamp    = rospy.Time.now()
        goal.target.header.frame_id = target_frame
        goal.target.pose            = Pose(Point(*target_pose[0:3]),
                                           Quaternion(
                                               *tfs.quaternion_from_euler(
                                                   *map(radians,
                                                        target_pose[3:6]))))
        goal.pointing_frame         = self._pointing_frame
        goal.min_duration           = self._min_duration
        goal.max_velocity           = self._max_velocity

        self._pose_head.send_goal(goal, feedback_cb=feedback_cb)

        rospy.loginfo('(PoseHeadClient) send goal[target_frame=%s,pointing_frame=%s]',
                      goal.target.header.frame_id, goal.pointing_frame)
    def cancel_goal(self):
        self._pose_head.cancel_goal()
