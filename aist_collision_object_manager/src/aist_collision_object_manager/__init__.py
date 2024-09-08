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
    def __init__(self, server='collision_object_manager'):
        super().__init__()

        self._touch_links = rospy.get_param('~touch_links', {})
        try:
            service = server + '/manage_collision_object'
            rospy.wait_for_service(service, timeout=5.0)
            self._send = rospy.ServiceProxy(service, ManageCollisionObject)
        except rospy.ROSException as e:
            rospy.logerr(e)

    def create_object(self, object_type, target_link, pose,
                      source_link='', object_id=''):
        source_subframe = 'base_link' if source_link == '' else \
                          source_link.rsplit('/', 1)[1]
        req = ManageCollisionObjectRequest()
        req.op              = ManageCollisionObjectRequest.ATTACH_OBJECT
        req.object_type     = object_type
        req.object_id       = object_id
        req.target_link     = target_link
        req.source_subframe = source_subframe
        req.pose            = pose
        req.touch_links     = self._get_touch_links(req.target_link)
        print('### target_link=%s, touch_links=%s' % (req.target_link,
                                                      req.touch_links))
        return self._send(req).success

    def attach_object(self, object_id, target_link, pose,
                      source_link='', preserve_ascendants=False):
        source_subframe = 'base_link' if source_link == '' else \
                          source_link.rsplit('/', 1)[1]
        req = ManageCollisionObjectRequest()
        req.op                  = ManageCollisionObjectRequest.ATTACH_OBJECT
        req.object_id           = object_id
        req.target_link         = target_link
        req.source_subframe     = source_subframe
        req.pose                = pose
        req.touch_links         = self._get_touch_links(req.target_link)
        req.preserve_ascendants = preserve_ascendants
        res = self._send(req)
        return res.retval if res.success else None

    def append_touch_links(self, object_id, touch_link):
        req = ManageCollisionObjectRequest()
        req.op          = ManageCollisionObjectRequest.APPEND_TOUCH_LINKS
        req.object_id   = object_id
        req.touch_links = self._touch_links.get(touch_link, [])
        return self._send(req).success

    def remove_touch_links(self, object_id, untouch_link):
        req = ManageCollisionObjectRequest()
        req.op          = ManageCollisionObjectRequest.REMOVE_TOUCH_LINKS
        req.object_id   = object_id
        req.touch_links = self._touch_links.get(untouch_link, [])
        return self._send(req).success

    def remove_object(self, object_id='', target_link=''):
        req = ManageCollisionObjectRequest()
        req.op          = ManageCollisionObjectRequest.REMOVE_OBJECT
        req.object_id   = object_id
        req.target_link = target_link
        return self._send(req).success

    def get_object_type(self, object_id):
        req           = ManageCollisionObjectRequest()
        req.op        = ManageCollisionObjectRequest.GET_OBJECT_TYPE
        req.object_id = object_id
        res = self._send(req)
        return res.retval if res.success else None

    def get_object_parent(self, object_id):
        req           = ManageCollisionObjectRequest()
        req.op        = ManageCollisionObjectRequest.GET_OBJECT_PARENT
        req.object_id = object_id
        res = self._send(req)
        return res.retval if res.success else None

    def _get_touch_links(self, target_link):
        tokens = target_link.rsplit('/', 1)
        if len(tokens) == 1:
            return self._touch_links.get(target_link, [])
        else:
            d = self._touch_links.get(tokens[0], {})
            print(d)
            return d.get(tokens[1], [])
