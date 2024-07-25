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
from actionlib                import SimpleActionClient
from actionlib_msgs.msg       import GoalStatus
from numpy                    import clip
from aist_fastening_tools.msg import (ScrewToolCommandAction,
                                      ScrewToolCommandGoal)
from std_msgs.msg             import Bool


######################################################################
#  class FasteningToolClient                                         #
######################################################################
class FasteningToolClient(object):
    def __init__(self, name, type,
                 base_link=None, tip_link=None, touch_links=None):
        object.__init__(self)
        self._name        = name
        self._type        = type
        self._base_link   = base_link if base_link else name + '/base_link'
        self._tip_link    = tip_link if tip_link else name + '/tip_link'
        self._touch_links = touch_links
        self._parameters  = {}

    @staticmethod
    def create(name, props):
        type_name = props.pop('type', None)
        if type_name is None:
            raise rospy.ROSException('(FasteningToolClient) no type specified for tool[%s]' % name)
        ClientClass = globals()[type_name]
        if rospy.get_param('use_real_robot', False):
            return ClientClass(name, **props)
        else:
            return ClientClass.simulated(name, **props)

    @property
    def name(self):
        return self._name

    @property
    def type(self):
        return self._type

    @property
    def base_link(self):
        return self._base_link

    @property
    def tip_link(self):
        return self._tip_link

    @property
    def touch_links(self):
        return self._touch_links

    @property
    def parameters(self):
        return self._parameters

    @parameters.setter
    def parameters(self, parameters):
        for key, value in parameters.items():
            self._parameters[key] = value

    def tighten(self, timeout=rospy.Duration()):
        pass

    def loosen(self, timeout=rospy.Duration()):
        pass

    def wait(self):
        return True

    def cancel(self):
        pass

######################################################################
#  class ScrewTool                                                   #
######################################################################
class ScrewTool(FasteningToolClient):
    def __init__(self, name, controller_ns,
                 base_link=None, tip_link=None, touch_links=None,
                 speed=1.0, retighten=True):
        super().__init__(name, 'screw_tool', base_link, tip_link, touch_links)

        action_ns = controller_ns + '/command'
        self._client = SimpleActionClient(action_ns, ScrewToolCommandAction)
        self._parameters = {'speed': speed, 'retighten': retighten}

        if not self._client.wait_for_server(timeout=rospy.Duration(5)):
            self._client = None
            rospy.logerr('(ScrewTool) failed to connect to server[%s]',
                         action_ns)

        rospy.loginfo('%s initialized.', action_ns)

    @staticmethod
    def simulated(name, controller_ns,
                  base_link=None, tip_link=None, touch_links=None, speed=1.0):
        return FasteningToolClient(name, 'screw_tool',
                                   base_link, tip_link, touch_links)

    def tighten(self, timeout=rospy.Duration(10)):
        self._send_command(self._parameters['speed'],
                           self._parameters['retighten'])
        return self.wait(timeout)

    def loosen(self, timeout=rospy.Duration(10)):
        self._send_command(-self._parameters['speed'], False, timeout)
        return self.wait(timeout)

    def wait(self, timeout=rospy.Duration(10)):
        if timeout < rospy.Duration():
            return False
        if not self._client.wait_for_result(timeout):
            self._client.cancel_goal()
            rospy.logerr('goal CANCELED because timeout[%.1f] has expired.',
                         timeout.to_sec())
            return False
        return self._client.get_state() == GoalStatus.SUCCEEDED

    def cancel(self):
        if self._client.get_state() in (GoalStatus.PENDING, GoalStatus.ACTIVE):
            self._client.cancel_goal()

    def _send_command(self, speed, retighten):
        self._client.send_goal(ScrewToolCommandGoal(speed, retighten))
