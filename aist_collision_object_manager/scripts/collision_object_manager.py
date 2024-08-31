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
import os, copy, rospy, rospkg
import numpy as np

from collections            import namedtuple
from tf                     import transformations as tfs
from tf2_ros                import TransformBroadcaster
from std_msgs.msg           import Header, ColorRGBA
from geometry_msgs.msg      import (Point, Vector3, Quaternion, Pose,
                                    Transform, TransformStamped)
from shape_msgs.msg         import Mesh, MeshTriangle, Plane, SolidPrimitive
from visualization_msgs.msg import Marker
from aist_msgs.srv          import (ManageCollisionObject,
                                    ManageCollisionObjectRequest,
                                    ManageCollisionObjectResponse,
                                    GetMeshResource, GetMeshResourceResponse)
from moveit_msgs.msg        import CollisionObject, AttachedCollisionObject
from moveit_commander       import planning_scene_interface as psi

try:
    from pyassimp import pyassimp
except:
    # support pyassimp > 3.0
    try:
        import pyassimp
    except:
        pyassimp = False
        print("Failed to import pyassimp, see https://github.com/moveit/moveit/issues/86 for more info")

#########################################################################
#  class CollisionObjectManager                                         #
#########################################################################
class CollisionObjectManager(object):
    ObjectProperties = namedtuple('ObjectProperties',
                                  ['primitives', 'primitive_poses',
                                   'visual_mesh_urls', 'visual_mesh_poses',
                                   'visual_mesh_scales', 'visual_mesh_colors',
                                   'collision_meshes', 'collision_mesh_poses',
                                   'collision_mesh_scales',
                                   'subframe_names', 'subframe_poses'])
    ObjectInfo = namedtuple('ObjectInfo',
                            ['type', 'subframe_transforms', 'markers'])

    def __init__(self, ns='', synchronous=True):
        super().__init__()

        PRIMITIVES = {'BOX':      SolidPrimitive.BOX,
                      'SPHERE':   SolidPrimitive.SPHERE,
                      'CYLINDER': SolidPrimitive.CYLINDER,
                      'CONE':     SolidPrimitive.CONE}

        # Load object properties from database.
        self._obj_props_dict = {}
        for type, props in rospy.get_param('~object_properties', {}).items():
            obj_props = CollisionObjectManager.ObjectProperties([], [],
                                                                [], [], [], [],
                                                                [], [], [],
                                                                [], [])
            for primitive in props.get('primitives', []):
                primitive_pose = primitive['pose']
                obj_props.primitives.append(SolidPrimitive(
                    type=PRIMITIVES[primitive['type']],
                    dimensions=primitive['dimensions']))
                obj_props.primitive_poses.append(
                    Pose(Point(*primitive_pose[0:3]),
                         Quaternion(*tfs.quaternion_from_euler(
                                        *np.radians(primitive_pose[3:6])))))

            for subframe_name, subframe_pose in props.get('subframes',
                                                          {}).items():
                obj_props.subframe_names.append(subframe_name)
                obj_props.subframe_poses.append(
                    Pose(Point(*subframe_pose[0:3]),
                         Quaternion(*tfs.quaternion_from_euler(
                                        *np.radians(subframe_pose[3:6])))))

            for mesh in props.get('visual_meshes', []):
                obj_props.visual_mesh_urls.append(mesh['url'])
                mesh_pose = mesh['pose']
                obj_props.visual_mesh_poses.append(
                    Pose(Point(*mesh_pose[0:3]),
                         Quaternion(*tfs.quaternion_from_euler(
                             *np.radians(mesh_pose[3:6])))))
                obj_props.visual_mesh_scales.append(Vector3(*mesh['scale']))
                obj_props.visual_mesh_colors.append(ColorRGBA(*mesh['color']))

            for mesh in props.get('collision_meshes', []):
                obj_props.collision_meshes.append(
                    self._load_mesh(mesh['url'], mesh['scale']))
                mesh_pose = mesh['pose']
                obj_props.collision_mesh_poses.append(
                    Pose(Point(*mesh_pose[0:3]),
                         Quaternion(*tfs.quaternion_from_euler(
                             *np.radians(mesh_pose[3:6])))))
                obj_props.collision_mesh_scales.append(Vector3(*mesh['scale']))

            self._obj_props_dict[type] = obj_props
            rospy.loginfo('(CollisionObjectManager) properties of object[%s] loaded', type)

        self._psi             = psi.PlanningSceneInterface(ns, synchronous)
        self._obj_info_dict   = {}
        self._marker_id_min   = 0
        self._marker_id_lists = {}
        self._marker_pub      = rospy.Publisher("~collision_marker",
                                                Marker, queue_size=10)
        self._broadcaster = TransformBroadcaster()
        self._timer       = rospy.Timer(rospy.Duration(0.1),
                                        self._subframes_and_markers_cb)
        self._get_mesh_resource \
            = rospy.Service('~get_mesh_resource', GetMeshResource,
                            self._get_mesh_resource_cb)
        self._manage_collision_object \
            = rospy.Service('~manage_collision_object', ManageCollisionObject,
                            self._manage_collision_object_cb)

    # timer callbacks
    def _subframes_and_markers_cb(self, event):
        for obj_info in self._obj_info_dict.values():
            for subframe_transform in obj_info.subframe_transforms:
                subframe_transform.header.stamp = rospy.Time.now()
                self._broadcaster.sendTransform(subframe_transform)
            for marker in obj_info.markers:
                self._marker_pub.publish(marker)

    # service callbacks
    def _get_mesh_resource_cb(self, req):
        res = GetMeshResourceResponse()
        res.mesh_resource = req.mesh_resource
        for obj_props in self._obj_props_dict.values():
            if req.mesh_resource in obj_props.visual_mesh_urls:
                with open(self._url_to_filepath(req.mesh_resource), 'rb') as f:
                    res.data = f.read()
                rospy.loginfo('(ObjectDatabaseServer) Send response to GetMeshResource request for the mesh_url[%s]', req.mesh_resource)
                return res
        rospy.logerr('(ObjectDatabaseServer) Received GetMeshResource request with unknown mesh_url[%s]', req.mesh_resource)
        return res

    def _manage_collision_object_cb(self, req):
        res = ManageCollisionObjectResponse(True, '')

        if req.op == ManageCollisionObjectRequest.REMOVE_OBJECT:
            self._remove_object(req.object_id, req.attach_link)
            return res
        elif req.op == ManageCollisionObjectRequest.GET_OBJECT_TYPE:
            obj_info = self._obj_info_dict.get(req.object_id, None)
            if obj_info is None:
                rospy.logerr('(CollisionObjectManager) unknown attached object[%s]',
                             req.object_id)
                res.success = False
                return res
            res.retval = obj_info.type
            return res

        if req.object_type == '':
            aco = self._psi.get_attached_objects([req.object_id]) \
                           .get(req.object_id, None)
            if aco is None:
                rospy.logerr('(CollisionObjectManager) unknown attached object[%s]',
                             req.object_id)
                res.success = False
                return res
        else:
            aco = self._create_object(req.object_type, req.object_id)

        if req.op == ManageCollisionObjectRequest.ATTACH_OBJECT:
            res.retval = self._attach_object(aco, req.attach_link,
                                             req.pose, req.touch_links)
        elif req.op == ManageCollisionObjectRequest.APPEND_TOUCH_LINKS:
            self._set_touch_links(aco, list(set(aco.touch_links) |
                                            set(req.touch_links)))
        elif req.op == ManageCollisionObjectRequest.REMOVE_TOUCH_LINKS:
            self._set_touch_links(aco, list(set(aco.touch_links) -
                                            set(req.touch_links)))
        else:
            rospy.logerr('(CollisionObjectManager) unknown operation[%d]',
                         req.op)
            res.success = False
        return res

    # operations
    def _create_object(self, object_type, object_id):
        # Create an attached collision object.
        obj_props = self._obj_props_dict[object_type]
        aco = AttachedCollisionObject()
        aco.object.id = object_id if object_id != '' else object_type
        if obj_props.collision_meshes != []:
            aco.object.meshes     = obj_props.collision_meshes
            aco.object.mesh_poses = obj_props.collision_mesh_poses
        else:
            aco.object.primitives      = obj_props.primitives
            aco.object.primitive_poses = obj_props.primitive_poses
        aco.object.subframe_names = obj_props.subframe_names
        aco.object.subframe_poses = obj_props.subframe_poses
        aco.object.operation      = CollisionObject.ADD

        # Create info for this object.
        obj_info = CollisionObjectManager.ObjectInfo(object_type, [], [])

        # Create subframe transforms.
        base_link = aco.object.id + '/base_link'
        obj_info.subframe_transforms.append(
            TransformStamped(Header(), base_link,
                             Transform(Vector3(0, 0, 0),
                                       Quaternion(0, 0, 0, 1))))
        for subframe_name, subframe_pose in zip(obj_props.subframe_names,
                                                obj_props.subframe_poses):
            obj_info.subframe_transforms.append(
                TransformStamped(Header(frame_id=base_link),
                                 aco.object.id + '/' + subframe_name,
                                 Transform(Vector3(subframe_pose.position.x,
                                                   subframe_pose.position.y,
                                                   subframe_pose.position.z),
                                           Quaternion(
                                               subframe_pose.orientation.x,
                                               subframe_pose.orientation.y,
                                               subframe_pose.orientation.z,
                                               subframe_pose.orientation.w))))

        # Create new marker IDs if not exit for this object.
        if aco.object.id not in self._marker_id_lists:
            self._marker_id_lists[aco.object.id] \
              = self._generate_marker_id_list(len(obj_props.visual_mesh_urls))

        # Create markers for visualization.
        for mesh_url, mesh_pose, mesh_scale, mesh_color, marker_id \
            in zip(obj_props.visual_mesh_urls,   obj_props.visual_mesh_poses,
                   obj_props.visual_mesh_scales, obj_props.visual_mesh_colors,
                   self._marker_id_lists[aco.object.id]):
            marker = Marker()
            marker.header.frame_id = base_link
            marker.ns              = ''
            marker.id              = marker_id
            marker.type            = marker.MESH_RESOURCE
            marker.action          = Marker.ADD
            marker.pose            = mesh_pose
            marker.scale           = mesh_scale
            marker.color           = mesh_color
            marker.lifetime        = rospy.Duration(0)
            marker.frame_locked    = False
            marker.mesh_resource   = mesh_url
            obj_info.markers.append(marker)

        # Store object info.
        self._obj_info_dict[aco.object.id] = obj_info

        rospy.loginfo('(CollisionObjectManager) created object with ID[%s] of type[%s]',
                      aco.object.id, object_type)
        return aco

    def _attach_object(self, aco, attach_link, pose, touch_links):
        detach_link = aco.link_name
        aco.object.header.frame_id = attach_link
        aco.object.pose            = pose
        aco.object.operation       = CollisionObject.ADD
        self._psi.attach_object(aco, attach_link, touch_links)

        rospy.loginfo("(CollisionObjectManager) attached '%s' to '%s' with touch_links%s",
                      aco.object.id, aco.link_name, aco.touch_links)

        # Replace the transform from the object base_link to the attached link.
        self._obj_info_dict[aco.object.id].subframe_transforms[0] \
            = TransformStamped(aco.object.header,
                               aco.object.id + '/base_link',
                               Transform(Vector3(pose.position.x,
                                                 pose.position.y,
                                                 pose.position.z),
                                         Quaternion(pose.orientation.x,
                                                    pose.orientation.y,
                                                    pose.orientation.z,
                                                    pose.orientation.w)))

        return detach_link

    def _set_touch_links(self, aco, touch_links):
        self._psi.attach_object(aco, touch_links=touch_links)
        rospy.loginfo("(CollisionObjectManager) set touch links %s to '%s'(attached to '%s')",
                      aco.touch_links, aco.object.id, aco.link_name)

    def _remove_object(self, object_id, attach_link):
        if object_id != '':
            self._delete_markers_and_subframes(object_id)
        elif attach_link != '':
            object_id = None
            for aco_id, aco in self._psi.get_attached_objects().items():
                if aco.link_name == attach_link:
                    self._delete_markers_and_subframes(aco_id)
        else:
            object_id = None
            attach_link = None
            for aco_id in self._psi.get_attached_objects().keys():
                self._delete_markers_and_subframes(aco_id)
        self._psi.remove_attached_object(attach_link, object_id)
        self._psi.remove_world_object(object_id)

    # utilities
    def _generate_marker_id_list(self, n):
        marker_id_list = []
        for i in range(n):
            marker_id_list.append(self._marker_id_min)
            self._marker_id_min += 1
        return marker_id_list

    def _delete_markers_and_subframes(self, object_id):
        for marker in self._obj_info_dict[object_id].markers:
            marker.action = Marker.DELETE
            self._marker_pub.publish(marker)
        del self._obj_info_dict[object_id]
        rospy.loginfo("(CollisionObjectManager) removed '%s'", object_id)

    def _load_mesh(self, url, scale=(0.001, 0.001, 0.001)):
        try:
            scene = pyassimp.load(self._url_to_filepath(url))
            if not scene.meshes or len(scene.meshes) == 0:
                raise Exception("There are no meshes in the file")
            if len(scene.meshes[0].faces) == 0:
                raise Exception("There are no faces in the mesh")

            mesh = Mesh()
            first_face = scene.meshes[0].faces[0]
            if hasattr(first_face, '__len__'):
                for face in scene.meshes[0].faces:
                    if len(face) == 3:
                        triangle = MeshTriangle()
                        triangle.vertex_indices = [face[0], face[1], face[2]]
                        mesh.triangles.append(triangle)
            elif hasattr(first_face, 'indices'):
                for face in scene.meshes[0].faces:
                    if len(face.indices) == 3:
                        triangle = MeshTriangle()
                        triangle.vertex_indices = [face.indices[0],
                                                   face.indices[1],
                                                   face.indices[2]]
                        mesh.triangles.append(triangle)
            else:
                raise Exception("Unable to build triangles from mesh due to mesh object structure")
            for vertex in scene.meshes[0].vertices:
                mesh.vertices.append(Point(vertex[0]*scale[0],
                                           vertex[1]*scale[1],
                                           vertex[2]*scale[2]))
            pyassimp.release(scene)
            return mesh
        except Exception as e:
            rospy.logerr('(CollisionObjectManager) failed to load mesh: %s', e)
            return None

    def _url_to_filepath(self, url):
        tokens = url.split('/')
        if len(tokens) < 2 or tokens[0] != 'package:' or tokens[1] != '':
            raise('Illegal URL: ' + url)
        return os.path.join(rospkg.RosPack().get_path(tokens[2]), *tokens[3:])

#########################################################################
#  Entry point                                                          #
#########################################################################
if __name__ == '__main__':

  rospy.init_node('collision_object_manager', anonymous=True)

  server = CollisionObjectManager()
  rospy.spin()
