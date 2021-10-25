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
import dynamic_reconfigure.client
from std_srvs import srv as ssrv

#########################################################################
#  class DepthFilterClient                                              #
#########################################################################
class DepthFilterClient(object):
    def __init__(self, server='depth_filter'):
        super(DepthFilterClient, self).__init__()
        self._saveBG     = rospy.ServiceProxy(server + '/saveBG',  ssrv.Trigger)
        self._capture    = rospy.ServiceProxy(server + '/capture', ssrv.Trigger)
        self._dyn_reconf = dynamic_reconfigure.client.Client(server,
                                                             timeout=5.0)

    def saveBG(self):
        return self._saveBG().success

    def capture(self):
        return self._capture().success

    @property
    def background_threshold(self):
        conf = self._dyn_reconf.get_configuration()
        return conf['thresh_bg']

    @background_threshold.setter
    def background_threshold(self, value):
        self._dyn_reconf.update_configuration({'thresh_bg': value})

    @property
    def depth_range(self):
        conf = self._dyn_reconf.get_configuration()
        return (conf['near'], conf['far'])

    @depth_range.setter
    def depth_range(self, range):
        self._dyn_reconf.update_configuration({'near': range[0],
                                               'far':  range[1]})

    @property
    def roi(self):
        conf = self._dyn_reconf.get_configuration()
        return (conf['left'], conf['top'], conf['right'], conf['bottom'])

    @roi.setter
    def roi(self, bbox):
        self._dyn_reconf.update_configuration({'left'  : bbox[0],
                                               'top'   : bbox[1],
                                               'right' : bbox[2],
                                               'bottom': bbox[3]})

    @property
    def window_radius(self):
        conf = self._dyn_reconf.get_configuration()
        return conf['window_radius']

    @window_radius.setter
    def window_radius(self, value):
        self._dyn_reconf.update_configuration({'window_radius': value})

    @property
    def scale(self):
        conf = self._dyn_reconf.get_configuration()
        return conf['scale']

    @scale.setter
    def scale(self, value):
        self._dyn_reconf.update_configuration({'scale': value})
