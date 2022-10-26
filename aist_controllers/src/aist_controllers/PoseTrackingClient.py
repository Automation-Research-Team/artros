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
from actionlib            import SimpleActionClient
from aist_controllers.msg import PoseTrackingAction, PoseTrackingGoal

######################################################################
#  class PoseTrackingClient                                          #
######################################################################
class PoseTrackingClient(object):
    def __init__(self, server="/pose_tracking_servo"):
        super(PoseTrackingClient, self).__init__()

        self._pose_tracking = SimpleActionClient(server + '/pose_tracking',
                                                 PoseTrackingAction)

        if self._pose_tracking.wait_for_server(timeout=rospy.Duration(5)):
            rospy.loginfo('(PoseTrackingClient) connected to server[%s]',
                          server + '/pose_tracking')
        else:
            self._pose_tracking = None
            rospy.logerr('(PoseTrackingClient) failed to connect to server[%s]',
                         server + '/pose_tracking')

    @property
    def is_connected(self):
        return self._pose_tracking is not None

    def send_goal(self, target_pose,
                  lin_tol=(0, 0, 0), rot_tol=0, feedback_cb=None):
        self._pose_tracking.send_goal(PoseTrackingGoal(target_pose=target_pose,
                                                       lin_tol=lin_tol,
                                                       rot_tol=rot_tol),
                                      feedback_cb=feedback_cb)
        rospy.loginfo('(PoseTrackingClient) send goal[target_frame=%s]',
                      target_pose.header.frame_id)

    def cancel_goal(self):
        self._pose_tracking.cancel_goal()

    def get_state(self):
        return self._pose_tracking.get_state()