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
import os, rospy, rospkg
import threading

from geometry_msgs.msg                         import (Vector3, Quaternion,
                                                       Transform,
                                                       TransformStamped)
from moveit_commander.planning_scene_interface import PlanningSceneInterface
from moveit_msgs.msg                           import CollisionObject
from visualization_msgs.msg                    import Marker
from std_msgs.msg                              import Header
from tf2_ros                                   import TransformBroadcaster
from aist_msgs.srv                             import GetObjectProperties

#########################################################################
#  class CollisionObjectManager                                         #
#########################################################################
class CollisionObjectManager(object):
    def __init__(self, ns='',
                 server='object_database_server', synchronous=True):
        super().__init__()

        self._psi                   = PlanningSceneInterface(ns, synchronous)
        self._touch_links           = rospy.get_param('~touch_links', {})
        self._marker_id_max         = 0
        self._subframe_transforms   = {}
        self._markers               = {}
        self._marker_pub            = rospy.Publisher("collision_marker",
                                                      Marker, queue_size=10)
        self._get_object_properties = rospy.ServiceProxy(
                                          server + '/get_object_properties',
                                          GetObjectProperties)
        self._lock                   = threading.Lock()
        th = threading.Thread(target=self._subframes_and_markers_thread)
        th.daemon = True
        th.start()

    def create_object(self, object_name, pose, object_id=None):
        object_props = self._get_object_properties(object_name).properties

        # Create and attach a collision object.
        co = CollisionObject()
        co.header.frame_id = pose.header.frame_id
        co.pose            = pose.pose
        co.id              = object_name if object_id is None else object_id
        if object_props.collision_meshes != []:
            co.meshes     = object_props.collision_meshes
            co.mesh_poses = object_props.collision_mesh_poses
        else:
            co.primitives      = object_props.primitives
            co.primitive_poses = object_props.primitive_poses
        co.subframe_names = object_props.subframe_names
        co.subframe_poses = object_props.subframe_poses
        co.operation      = CollisionObject.ADD
        self._psi.attach_object(co, co.header.frame_id,
                                self.get_touch_links(co.header.frame_id))
        rospy.loginfo('created collision object[%s] and attached to %s',
                      co.id, co.header.frame_id)

        # Create subframe transforms.
        base_link = co.id + '/base_link'
        subframe_transforms = []
        T = TransformStamped(co.header, base_link,
                             Transform(Vector3(pose.pose.position.x,
                                               pose.pose.position.y,
                                               pose.pose.position.z),
                                       Quaternion(pose.pose.orientation.x,
                                                  pose.pose.orientation.y,
                                                  pose.pose.orientation.z,
                                                  pose.pose.orientation.w)))
        subframe_transforms.append(T)
        for subframe_name, subframe_pose in zip(object_props.subframe_names,
                                                object_props.subframe_poses):
            T = TransformStamped(Header(frame_id=base_link),
                                 co.id + '/' + subframe_name,
                                 Transform(Vector3(subframe_pose.position.x,
                                                   subframe_pose.position.y,
                                                   subframe_pose.position.z),
                                           Quaternion(
                                               subframe_pose.orientation.x,
                                               subframe_pose.orientation.y,
                                               subframe_pose.orientation.z,
                                               subframe_pose.orientation.w)))
            subframe_transforms.append(T)

        # Create markers for visualization.
        markers = []
        for mesh_url, mesh_pose, mesh_scale, mesh_color \
            in zip(object_props.visual_mesh_urls,
                   object_props.visual_mesh_poses,
                   object_props.visual_mesh_scales,
                   object_props.visual_mesh_colors):
            marker = Marker()
            marker.header.frame_id = base_link
            marker.ns              = ''
            marker.id              = self._marker_id_max
            marker.type            = marker.MESH_RESOURCE
            marker.action          = Marker.ADD
            marker.pose            = mesh_pose
            marker.scale           = mesh_scale
            marker.color           = mesh_color
            marker.lifetime        = rospy.Duration(0)
            marker.frame_locked    = False
            marker.mesh_resource   = mesh_url
            markers.append(marker)
            self._marker_id_max += 1

        # Keep created subframe transforms and markers.
        with self._lock:
            self._subframe_transforms[co.id] = subframe_transforms
            self._markers[co.id] = markers

    def attach_object(self, object_id, pose):
        aco = self._psi.get_attached_objects([object_id]).get(object_id, None)
        if aco is None:
            rospy.logerr('unknown attached object[%s]', object_id)
            return None
        old_link_name = aco.link_name
        aco.object.header.frame_id = pose.header.frame_id
        aco.object.pose            = pose.pose
        aco.object.operation       = CollisionObject.ADD
        self._psi.attach_object(aco, pose.header.frame_id,
                                self.get_touch_links(pose.header.frame_id))
        rospy.loginfo('attached %s to %s with touch_links%s',
                      aco.object.id, aco.link_name, aco.touch_links)

        # Publish visualization markers again.
        for marker in self._markers[object_id]:
            self._marker_pub.publish(marker)

        # Replace the transform from the object base_link to the attached link.
        with self._lock:
            self._subframe_transforms[object_id][0] \
                = TransformStamped(aco.object.header,
                                   aco.object.id + '/base_link',
                                   Transform(Vector3(pose.pose.position.x,
                                                     pose.pose.position.y,
                                                     pose.pose.position.z),
                                             Quaternion(
                                                 pose.pose.orientation.x,
                                                 pose.pose.orientation.y,
                                                 pose.pose.orientation.z,
                                                 pose.pose.orientation.w)))

        return old_link_name

    def append_touch_links(self, object_id, link_to_touch):
        aco = self._psi.get_attached_objects([object_id]).get(object_id, None)
        if aco is None:
            rospy.logerr('unknown attached object[%s]', object_id)
            return

        self._psi.attach_object(aco,
                                touch_links=list(
                                    set(aco.touch_links) |
                                    set(self.get_touch_links(link_to_touch))))
        rospy.loginfo('appended touch links to %s attached to %s resulting in touch_links=%s',
                      aco.object.id, aco.link_name, aco.touch_links)

    def remove_touch_links(self, object_id, link_to_detach):
        aco = self._psi.get_attached_objects([object_id]).get(object_id, None)
        if aco is None:
            rospy.logerr('unknown attached object[%s]', object_id)
            return

        self._psi.attach_object(aco,
                                touch_links=list(
                                    set(aco.touch_links) -
                                    set(self.get_touch_links(link_to_detach))))
        rospy.loginfo('removed touch_links from %s attached to %s resulting in touch_links=%s',
                      aco.object.id, aco.link_name, aco.touch_links)

    def remove_attached_object(self, link=None, object_id=None):
        if object_id is not None:
            markers = self._markers.get(object_id, None)
            if markers is None:
                rospy.logerr('unknown attached object[%s]', object_id)
                return
            for marker in markers:
                self._delete_marker(marker.id)
            with self._lock:
                del self._subframe_transforms[object_id]
                del self._markers[object_id]
        elif link is not None:
            for object_id, aco in self.get_attached_objects().items():
                if aco.link_name == link:
                    for marker in self._markers[object_id]:
                        self._delete_marker(marker.id)
                    with self._lock:
                        del self._subframe_transforms[object_id]
                        del self._markers[object_id]
        else:
            for markers in self._markers.values():
                for marker in markers:
                    self._delete_marker(marker.id)
            self._marker_id_max = 0
            with self._lock:
                self._subframe_transforms.clear()
                self._markers.clear()
        self._psi.remove_attached_object(link, object_id)
        self._psi.remove_world_object(object_id)

    def find_attached_objects(self, link):
        return [aco for aco in self._psi.get_attached_objects().values()
                if aco.link_name == link]

    def print_object_info(self, object_id):
        aco = self._psi.get_attached_objects([object_id]).get(object_id, None)
        if aco is None:
            rospy.logerr('unknown attached object[%s]', object_id)
            return
        rospy.loginfo('%s: attached to %s with touch_links%s',
                      aco.object.id, aco.link_name, aco.touch_links)

    def get_touch_links(self, link_to_touch):
        return self._touch_links.get(link_to_touch, [])

    # visualization marker stuffs
    def _delete_marker(self, marker_id):
        marker = Marker()
        marker.id     = marker_id
        marker.action = Marker.DELETE
        self._marker_pub.publish(marker)

    # subframe transforms stuff
    def _subframes_and_markers_thread(self):
        self._broadcaster = TransformBroadcaster()
        self._timer       = rospy.Timer(rospy.Duration(0.1),
                                        self._subframes_and_markers_cb)
        rospy.spin()

    def _subframes_and_markers_cb(self, event):
        with self._lock:
            for subframe_transforms in self._subframe_transforms.values():
                for subframe_transform in subframe_transforms:
                    subframe_transform.header.stamp = rospy.Time.now()
                    self._broadcaster.sendTransform(subframe_transform)
            for markers in self._markers.values():
                for marker in markers:
                    self._marker_pub.publish(marker)
