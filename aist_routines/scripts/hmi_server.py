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
import rospy
from aist_msgs.msg     import (RequestHelpAction, RequestHelpGoal,
                               RequestHelpResult, RequestHelpFeedback,
                               RequestHelp, Pointing)
from actionlib         import SimpleActionServer
from geometry_msgs.msg import (QuaternionStamped, PoseStamped, PointStamped,
                               Vector3Stamped, Point, Quaternion, Vector3)

######################################################################
#  class HMIServer                                                   #
######################################################################
class HMIServer(object):
    _Pointing    = ('NO_RES', 'SWEEP_RES', 'RECAPTURE_RES')

    def __init__(self):
        super(HMIServer, self).__init__()

        self._seq              = 0
        self._no_req           = RequestHelp(robot_name='unknown_robot_name',
                                              item_id='unknown_part_ID',
                                              request=RequestHelp.NO_REQ,
                                              message='no_requests')
        self._curr_req         = self._no_req
        self._request_help_pub = rospy.Publisher('/help', RequestHelp,
                                                 queue_size=10)
        self._pointing_sub     = rospy.Subscriber('/pointing', Pointing,
                                                  self._pointing_cb)
        self._request_help_srv = SimpleActionServer('~request_help',
                                                    RequestHelpAction,
                                                    auto_start=False)

        self._request_help_srv.register_goal_callback(self._goal_cb)
        self._request_help_srv.register_preempt_callback(self._preempt_cb)
        self._request_help_srv.start()

    def run(self):
        rate = rospy.Rate(10)   # 10Hz
        while not rospy.is_shutdown():
            self._curr_req.pose.header.seq = self._seq
            self._request_help_pub.publish(self._curr_req)
            self._seq += 1
            rate.sleep()

    def _pointing_cb(self, pointing_msg):
        """
        Receive response message with finger direction from VR side.
        Send feedback to the action client if pointing_state is NO_RES.
        Set state of the goal to SUCCEEDED, send the message as a result
        and revert _curr_req to _no_req, otherwise.

        @type  pointing_msg: finger_pointing_msgs.msg.pointing
        @param pointing_msg: finger direction response from VR side
        """
        pointing_msg.header.stamp = rospy.Time.now()
        if self._request_help_srv.is_active():
            if pointing_msg.pointing_state == Pointing.NO_RES:
                self._request_help_srv.publish_feedback(
                    RequestHelpFeedback(pointing_msg))
            else:
                self._request_help_srv.set_succeeded(RequestHelpResult(
                                                        pointing_msg))
                self._curr_req = self._no_req   # Revert to _no_req
                rospy.loginfo('(hmi_server) SUCCEEDED current goal[%s: pos=(%f %f %f), dir=(%f, %f, %f)]'
                              % (self._Pointing[pointing_msg.pointing_state],
                                 pointing_msg.finger_pos.x,
                                 pointing_msg.finger_pos.y,
                                 pointing_msg.finger_pos.z,
                                 pointing_msg.finger_dir.x,
                                 pointing_msg.finger_dir.y,
                                 pointing_msg.finger_dir.z))

    def _goal_cb(self):
        """
        Accept new goal from action client and store the help request from
        robot side in _curr_req.
        """
        self._curr_req = self._request_help_srv.accept_new_goal().request
        rospy.loginfo('(hmi_server) ACCPETED new goal for help request[%s, %s]',
                      self._curr_req.robot_name, self._curr_req.item_id)

    def _preempt_cb(self):
        """
        Set state of the goal to PREEMPTED and revert _curr_req to _no_req
        upon a cancel request from the action client.
        """
        pointing_msg = Pointing()
        pointing_msg.pointing_state = Pointing.NO_RES
        self._request_help_srv.set_preempted(RequestHelpResult(pointing_msg))
        self._curr_req = self._no_req           # Revert to _no_req
        rospy.loginfo('(hmi_server) PREEMPTED current goal')


#########################################################################
#  entry point                                                          #
#########################################################################
if __name__ == '__main__':
    try:
        rospy.init_node('hmi_server')
        hmi_srv = HMIServer()
        hmi_srv.run()
    except rospy.ROSInitException as e:
        rospy.logerr(e)
    except rospy.ROSInterruptException:
        pass
