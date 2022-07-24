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
from finger_pointing_msgs.msg import (RequestHelpAction, RequestHelpGoal,
                                      RequestHelpResult, RequestHelpFeedback,
                                      request_help, pointing)
from actionlib                import SimpleActionServer

######################################################################
#  class HMIServer                                                   #
######################################################################
class HMIServer(object):
    def __init__(self):
        super(HMIServer, self).__init__()

        self._no_req     = request_help(request=request_help.NO_REQ)
        self._curr_req   = self._no_req
        self._prev_state = pointing.NO_RES
        self._hmi_pub    = rospy.Publisher('request_help', request_help,
                                           queue_size=10)
        self._hmi_sub    = rospy.Subscriber('/pointing', pointing,
                                            self._pointing_cb)
        self._hmi_srv    = SimpleActionServer('~request_help',
                                              RequestHelpAction,
                                              auto_start=False)

        self._hmi_srv.register_goal_callback(self._goal_cb)
        self._hmi_srv.register_preempt_callback(self._preempt_cb)
        self._hmi_srv.start()

    def run(self):
        rate = rospy.Rate(10)   # 10Hz
        while not rospy.is_shutdown():
            self._hmi_pub.publish(self._curr_req)
            rate.sleep()

    def _pointing_cb(self, pointing_msg):
        pointing_msg.header.stamp = rospy.Time.now()
        if self._hmi_srv.is_active():
            self._hmi_srv.publish_feedback(RequestHelpFeedback(pointing_msg))
            if self._prev_state is pointing.NO_RES and \
               pointing_msg.pointing_state is not pointing.NO_RES:
                self._hmi_srv.set_succeeded(RequestHelpResult(pointing_msg))
                self._curr_req = self._no_req

        self._prev_state = pointing_msg.pointing_state

    def _goal_cb(self):
        self._curr_req = self._hmi_srv.accept_new_goal().request
        rospy.loginfo('(hmi_server) ACCPETED new goal[robot_name=%s, item_id=%s. request=%d]',
                      self._curr_req.robot_name, self._curr_req.item_id,
                      self._curr_req.request)

    def _preempt_cb(self):
        self._request.set_preempted()
        self._request = self._no_req
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
