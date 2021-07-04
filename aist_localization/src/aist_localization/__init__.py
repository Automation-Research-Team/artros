import rospy
import dynamic_reconfigure.client
import actionlib
from aist_localization import msg as lmsg
from operator          import itemgetter

#########################################################################
#  class LocalizationClient                                             #
#########################################################################
class LocalizationClient(object):
    _DefaultSettings = {
                           'Scene_Clustering_Level':              'Normal',
                           'Scene_Minimal_Cluster_Size':          200,
                           'Scene_Maximal_Cluster_Size':          3500000,
                           'Matching_Algorithm':                  'Surfaces',
                           'Model_Keypoints_Sampling':            'Medium',
                           'Local_Search_Radius':                 'Normal',
                           'Feature_fit_consideration_level':     15,
                           'Global_maximal_feature_fit_overflow': 20,
                           'Fine_Alignment_Iterations':           30,
                           'Fine_Alignment_Point_Set':            'Surface',
                           'Fine_Alignment_Point_Set_Sampling':   'Sampled',
                           'Projection_Tolerance':                100,
                           'Projection_Hidden_Part_Tolerance':    100,
                           'Overlap':                             15.0
                       }

    def __init__(self, server='localization'):
        super(LocalizationClient, self).__init__()

        self._in_plane          = rospy.get_param('~in_plane',          {})
        self._full_settings     = rospy.get_param('~full_settings',     {})
        self._in_plane_settings = rospy.get_param('~in_plane_settings', {})

        self._dyn_reconf = dynamic_reconfigure.client.Client(server,
                                                             timeout=5.0)
        self._localize   = actionlib.SimpleActionClient(server + '/localize',
                                                        lmsg.LocalizeAction)
        self._localize.wait_for_server()

    def set_setting(self, name, value):
        self.set_settings({name : value})
        return self.get_setting(name)

    def get_setting(self, name):
        return self.get_settings()[name]

    def set_settings(self, settings):
        self._dyn_reconf.update_configuration(settings)

    def get_settings(self):
        return self._dyn_reconf.get_configuration()

    def send_goal(self, model, nposes=1, poses2d=[]):
        goal = lmsg.LocalizeGoal()
        goal.object_name = model
        goal.in_plane    = model in self._in_plane
        goal.nposes      = nposes
        goal.poses2d     = poses2d
        goal.sideways    = False
        goal.x_offset    = 0.0
        goal_z_offset    = 0.0
        if model in self._in_plane_settings:
            in_plane_settings = self._in_plane_settings[model]
            if 'sideways' in in_plane_settings:
                goal.sideways = in_plane_settings['sideways']
            if 'x_offset' in in_plane_settings:
                goal.x_offset = in_plane_settings['x_offset']
            if 'z_offset' in in_plane_settings:
                goal.z_offset = in_plane_settings['z_offset']

        self.set_settings(LocalizationClient._DefaultSettings)
        if model in self._full_settings:
            self.set_settings(self._full_settings[model])

        self._poses    = []
        self._overlaps = []
        self._localize.send_goal(goal, feedback_cb=self._feedback_cb)

    def wait_for_result(self, timeout=rospy.Duration()):
        if (not self._localize.wait_for_result(timeout)):
            self._localize.cancel_goal()  # Cancel goal if timeout expired

        # Sort obtained poses in descending order of overlap values.
        pairs = sorted(zip(self._poses, self._overlaps),
                       key=itemgetter(1), reverse=True)
        if len(pairs):
            self._poses, self._overlaps = zip(*pairs)

        return (self._poses, self._overlaps)

    def _feedback_cb(self, feedback):
        self._poses.append(feedback.pose)
        self._overlaps.append(feedback.overlap)
