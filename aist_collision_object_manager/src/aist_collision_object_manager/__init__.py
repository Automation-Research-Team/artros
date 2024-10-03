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

from aist_msgs.srv import ManageCollisionObject, ManageCollisionObjectRequest

#########################################################################
#  class CollisionObjectManagerClient                                   #
#########################################################################
class CollisionObjectManagerClient(object):
    def __init__(self, listener, server='collision_object_manager'):
        super().__init__()

        self._listener    = listener
        try:
            service = server + '/manage_collision_object'
            rospy.wait_for_service(service, timeout=5.0)
            self._send = rospy.ServiceProxy(service, ManageCollisionObject)
        except rospy.ROSException as e:
            rospy.logerr(e)

    def create_object(self, object_type, frame_id, pose,
                      subframe_link='', object_id=''):
        req = ManageCollisionObjectRequest()
        req.op          = ManageCollisionObjectRequest.CREATE_OBJECT
        req.object_type = object_type
        req.object_id   = object_id if object_id != '' else object_type
        req.frame_id    = frame_id
        req.subframe    = CollisionObjectManagerClient \
                         ._subframe_name(subframe_link)
        req.pose        = pose
        return self._send(req).success

    def remove_object(self, object_id='', frame_id=''):
        req = ManageCollisionObjectRequest()
        req.op        = ManageCollisionObjectRequest.REMOVE_OBJECT
        req.object_id = object_id
        req.frame_id  = frame_id
        return self._send(req).success

    def attach_object(self, object_id, frame_id, pose, subframe_link=''):
        req = ManageCollisionObjectRequest()
        req.op        = ManageCollisionObjectRequest.ATTACH_OBJECT
        req.object_id = object_id
        req.frame_id  = frame_id
        req.subframe  = CollisionObjectManagerClient \
                       ._subframe_name(subframe_link)
        req.pose      = pose
        return self._send(req).success

    def detach_object(self, object_id, frame_id, pose, subframe_link=''):
        req = ManageCollisionObjectRequest()
        req.op        = ManageCollisionObjectRequest.DETACH_OBJECT
        req.object_id = object_id
        req.frame_id  = frame_id
        req.subframe  = CollisionObjectManagerClient \
                       ._subframe_name(subframe_link)
        req.pose      = pose
        return self._send(req).success

    def append_touch_links(self, object_id, touch_link):
        req = ManageCollisionObjectRequest()
        req.op        = ManageCollisionObjectRequest.APPEND_TOUCH_LINKS
        req.object_id = object_id
        req.frame_id  = touch_link
        return self._send(req).success

    def remove_touch_links(self, object_id, untouch_link):
        req = ManageCollisionObjectRequest()
        req.op        = ManageCollisionObjectRequest.REMOVE_TOUCH_LINKS
        req.object_id = object_id
        req.frame_id  = untouch_link
        return self._send(req).success

    def clean_touch_links(self, object_id):
        req = ManageCollisionObjectRequest()
        req.op        = ManageCollisionObjectRequest.CLEAN_TOUCH_LINKS
        req.object_id = object_id
        return self._send(req).success

    def get_object_info(self, object_id):
        req           = ManageCollisionObjectRequest()
        req.op        = ManageCollisionObjectRequest.GET_OBJECT_INFO
        req.object_id = object_id
        res = self._send(req)
        return res.info if res.success else None

    @staticmethod
    def _subframe_name(subframe_link):
        if subframe_link == '':
            return 'base_link'
        tokens = subframe_link.rsplit('/', 1)
        return tokens[0] if len(tokens) == 1 else tokens[1]
