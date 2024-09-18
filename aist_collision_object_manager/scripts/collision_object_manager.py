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
import os, copy, rospy, rospkg, threading
import numpy as np

from collections            import namedtuple
from tf                     import transformations as tfs
from tf2_ros                import (TransformBroadcaster, Buffer,
                                    TransformListener)
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
#  local functions                                                      #
#########################################################################
def _load_mesh(url, scale=(0.001, 0.001, 0.001)):
    try:
        scene = pyassimp.load(_url_to_filepath(url))
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

def _url_to_filepath(url):
    tokens = url.split('/')
    if len(tokens) < 2 or tokens[0] != 'package:' or tokens[1] != '':
        raise('Illegal URL: ' + url)
    return os.path.join(rospkg.RosPack().get_path(tokens[2]), *tokens[3:])

def _get_object_id_and_subframe(link_name):
    tokens = link_name.rsplit('/', 1)
    if len(tokens) == 2:
        return tokens[0], tokens[1]
    else:
        return '', link_name

def _pose_matrix(pose):
    return tfs.concatenate_matrices(tfs.translation_matrix([pose.position.x,
                                                            pose.position.y,
                                                            pose.position.z]),
                                    tfs.quaternion_matrix([pose.orientation.x,
                                                           pose.orientation.y,
                                                           pose.orientation.z,
                                                           pose.orientation.w]))

def _pose_from_matrix(T):
    return Pose(Point(*tfs.translation_from_matrix(T)),
                Quaternion(*tfs.quaternion_from_matrix(T)))

def _transform_pose(T, pose):
    return _pose_from_matrix(tfs.concatenate_matrices(T, _pose_matrix(pose)))

def _subframe_pose(co, subframe_name):
    return co.subframe_poses[co.subframe_names.index(subframe_name)]

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
    class InstanceProperties(object):
        def __init__(self, type, parent_link):
            self.type                = type
            self.parent_link         = parent_link
            self.subframe_transforms = []
            self.markers             = []

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
                obj_props.collision_meshes.append(_load_mesh(mesh['url'],
                                                             mesh['scale']))
                mesh_pose = mesh['pose']
                obj_props.collision_mesh_poses.append(
                    Pose(Point(*mesh_pose[0:3]),
                         Quaternion(*tfs.quaternion_from_euler(
                             *np.radians(mesh_pose[3:6])))))
                obj_props.collision_mesh_scales.append(Vector3(*mesh['scale']))

            self._obj_props_dict[type] = obj_props
            rospy.loginfo('(CollisionObjectManager) properties of object[%s] loaded', type)

        self._psi                 = psi.PlanningSceneInterface(ns, synchronous)
        self._instance_props_dict = {}
        self._marker_id_min       = 0
        self._marker_id_lists     = {}
        self._marker_pub          = rospy.Publisher('~collision_marker',
                                                    Marker, queue_size=10)
        self._buffer              = Buffer(None, debug=False)
        self._listener            = TransformListener(self._buffer)
        self._broadcaster         = TransformBroadcaster()
        self._lock                = threading.Lock()
        self._timer               = rospy.Timer(rospy.Duration(0.1),
                                                self._subframes_and_markers_cb)
        self._get_mesh_resource \
            = rospy.Service('~get_mesh_resource', GetMeshResource,
                            self._get_mesh_resource_cb)
        self._manage_collision_object \
            = rospy.Service('~manage_collision_object', ManageCollisionObject,
                            self._manage_collision_object_cb)

    def __del__(self):
        self._psi.clear()

    # timer callbacks
    def _subframes_and_markers_cb(self, event):
        with self._lock:
            for instance_props in self._instance_props_dict.values():
                for subframe_transform in instance_props.subframe_transforms:
                    subframe_transform.header.stamp = rospy.Time.now()
                    self._broadcaster.sendTransform(subframe_transform)
                for marker in instance_props.markers:
                    self._marker_pub.publish(marker)

    # service callbacks
    def _get_mesh_resource_cb(self, req):
        res = GetMeshResourceResponse()
        res.mesh_resource = req.mesh_resource
        for obj_props in self._obj_props_dict.values():
            if req.mesh_resource in obj_props.visual_mesh_urls:
                with open(_url_to_filepath(req.mesh_resource), 'rb') as f:
                    res.data = f.read()
                rospy.loginfo('(ObjectDatabaseServer) Send response to GetMeshResource request for the mesh_url[%s]', req.mesh_resource)
                return res
        rospy.logerr('(ObjectDatabaseServer) Received GetMeshResource request with unknown mesh_url[%s]', req.mesh_resource)
        return res

    def _manage_collision_object_cb(self, req):
        res = ManageCollisionObjectResponse(True, '')

        try:
            if req.op == ManageCollisionObjectRequest.CREATE_OBJECT:
                self._create_object(req.object_type, req.object_id,
                                    req.target_link, req.source_subframe,
                                    req.pose)
            elif req.op == ManageCollisionObjectRequest.REMOVE_OBJECT:
                self._remove_object(req.object_id, req.target_link)
            elif req.op == ManageCollisionObjectRequest.ATTACH_OBJECT:
                res.retval = self._attach_object(req.object_id,
                                                 req.target_link,
                                                 req.source_subframe,
                                                 req.pose, req.touch_links,
                                                 req.preserve_ascendants)
            elif req.op == ManageCollisionObjectRequest.DETACH_OBJECT:
                self._detach_object(req.object_id, req.target_link,
                                    req.source_subframe, req.pose)
            elif req.op == ManageCollisionObjectRequest.APPEND_TOUCH_LINKS:
                self._append_touch_links(req.object_id, req.touch_links)
            elif req.op == ManageCollisionObjectRequest.REMOVE_TOUCH_LINKS:
                self._remove_touch_links(req.object_id, req.touch_links)
            elif req.op == ManageCollisionObjectRequest.GET_OBJECT_TYPE:
                instance_props = self._instance_props_dict.get(req.object_id,
                                                               None)
                if instance_props is None:
                    raise Exception('unknown object[%s]' % req.object_id)
                res.retval = instance_props.type
            elif req.op == ManageCollisionObjectRequest.GET_OBJECT_PARENT:
                instance_props = self._instance_props_dict.get(req.object_id,
                                                               None)
                if instance_props is None:
                    raise Exception('unknown object[%s]' % req.object_id)
                res.retval = instance_props.parent_link
            else:
                raise Exception('unknown operation[%d]' % req.op)
        except Exception as e:
            rospy.logerr('(CollisionObjectManager) %s', e)
            res.success = False

        return res

    # operations
    def _create_object(self, object_type,
                       object_id, target_link, source_subframe, pose):
        obj_props = self._obj_props_dict.get(object_type, None)
        if obj_props is None:
            raise Exception('unknown object type[%s]' % req.object_type)

        # Create an attached collision object.
        co = CollisionObject()
        co.id = object_id if object_id != '' else object_type
        if obj_props.collision_meshes != []:
            co.meshes     = obj_props.collision_meshes
            co.mesh_poses = obj_props.collision_mesh_poses
        else:
            co.primitives      = obj_props.primitives
            co.primitive_poses = obj_props.primitive_poses
        co.subframe_names = obj_props.subframe_names
        co.subframe_poses = obj_props.subframe_poses
        co.operation      = CollisionObject.ADD
        if source_subframe != 'base_link':
            pose = _pose_from_matrix(
                       tfs.concatenate_matrices(
                           _pose_matrix(pose),
                           tfs.inverse_matrix(
                               _pose_matrix(_subframe_pose(co,
                                                           source_subframe)))))
        co.header.frame_id = target_link
        co.pose            = pose
        self._psi.add_object(co)

        # Create info for this object.
        instance_props = CollisionObjectManager.InstanceProperties(object_type,
                                                                   target_link)

        # Create subframe transforms.
        base_link = co.id + '/base_link'
        instance_props.subframe_transforms.append(
            TransformStamped(Header(frame_id=target_link), base_link,
                             Transform(Vector3(pose.position.x,
                                               pose.position.y,
                                               pose.position.z),
                                       Quaternion(pose.orientation.x,
                                                  pose.orientation.y,
                                                  pose.orientation.z,
                                                  pose.orientation.w))))
        for subframe_name, subframe_pose in zip(obj_props.subframe_names,
                                                obj_props.subframe_poses):
            instance_props.subframe_transforms.append(
                TransformStamped(Header(frame_id=base_link),
                                 co.id + '/' + subframe_name,
                                 Transform(Vector3(subframe_pose.position.x,
                                                   subframe_pose.position.y,
                                                   subframe_pose.position.z),
                                           Quaternion(
                                               subframe_pose.orientation.x,
                                               subframe_pose.orientation.y,
                                               subframe_pose.orientation.z,
                                               subframe_pose.orientation.w))))

        # Create new marker IDs if not exit for this object.
        if co.id not in self._marker_id_lists:
            self._marker_id_lists[co.id] \
              = self._generate_marker_id_list(len(obj_props.visual_mesh_urls))

        # Create markers for visualization.
        for mesh_url, mesh_pose, mesh_scale, mesh_color, marker_id \
            in zip(obj_props.visual_mesh_urls,   obj_props.visual_mesh_poses,
                   obj_props.visual_mesh_scales, obj_props.visual_mesh_colors,
                   self._marker_id_lists[co.id]):
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
            instance_props.markers.append(marker)

        # Store object info.
        with self._lock:
            self._instance_props_dict[co.id] = instance_props

        rospy.loginfo("(CollisionObjectManager) created object '%s' of type[%s]",
                      co.id, object_type)

    def _remove_object(self, object_id, target_link):
        if object_id != '':
            self._delete_markers_and_subframes(object_id)
        elif target_link != '':
            object_id = None
            for aco_id, aco in self._psi.get_attached_objects().items():
                if aco.link_name == target_link:
                    self._delete_markers_and_subframes(aco_id)
        else:
            object_id = None
            target_link = None
            for co_id in self._psi.get_objects().keys():
                self._delete_markers_and_subframes(co_id)
        self._psi.remove_attached_object(target_link, object_id)
        self._psi.remove_world_object(object_id)

    def _attach_object(self, object_id, target_link, source_subframe, pose,
                       touch_links, preserve_ascendants):
        aco = AttachedCollisionObject()
        aco.object = self._get_object(object_id)
        if aco.object is None:
            raise Exception("unknown collision object '%s'" % object_id)

        # If any subframe of the object other than 'base_link' is to be
        # attached to 'target_link', transform the given pose to that
        # with respect to 'base_link'.
        if source_subframe != 'base_link':
            pose = _pose_from_matrix(
                       tfs.concatenate_matrices(
                           _pose_matrix(pose),
                           tfs.inverse_matrix(
                               _pose_matrix(_subframe_pose(aco.object,
                                                           source_subframe)))))

        # Get information of the given collision object and set target link.
        instance_props = self._instance_props_dict[aco.object.id]

        # Replace the transform from the object base_link to the attached link.
        instance_props.subframe_transforms[0] \
            = TransformStamped(Header(frame_id=target_link),
                               aco.object.id + '/base_link',
                               Transform(Vector3(pose.position.x,
                                                 pose.position.y,
                                                 pose.position.z),
                                         Quaternion(pose.orientation.x,
                                                    pose.orientation.y,
                                                    pose.orientation.z,
                                                    pose.orientation.w)))

        # If 'target_link' belongs to any other collision object, replace it
        # with the link to which the object attached.
        target_link, pose = self._fix_target_link_and_pose(target_link, pose)

        # Keep the current attach link
        # and set the new one to the specified object and its descendants.
        self._set_target_link(aco, target_link, pose, touch_links)

        # Keep the current parent link of this collision object
        # and set the new one.
        old_parent_link = instance_props.parent_link
        instance_props.parent_link = target_link
        return old_parent_link

    def _detach_object(self, object_id, target_link, source_subframe, pose):
        aco = self._get_attached_object(object_id)
        if aco is None:
            return

        old_link_name = aco.link_name
        self._psi.remove_attached_object(None, aco.object.id)
        rospy.loginfo("(CollisionObjectManager) detach '%s' from '%s'",
                      aco.object.id, old_link_name)

    def _append_touch_links(self, object_id, touch_links):
        aco = self._get_attached_object(object_id)
        if aco is None:
            return
        self._psi.attach_object(aco, touch_links=list(set(aco.touch_links) |
                                                      set(touch_links)))
        rospy.loginfo("(CollisionObjectManager) set touch links%s to '%s' attached to '%s'",
                      aco.touch_links, aco.object.id, aco.link_name)

    def _remove_touch_links(self, object_id, touch_links):
        aco = self._get_attached_object(object_id)
        if aco is None:
            return
        self._psi.attach_object(aco, touch_links=list(set(aco.touch_links) -
                                                      set(touch_links)))
        rospy.loginfo("(CollisionObjectManager) set touch links%s to '%s' attached to '%s'",
                      aco.touch_links, aco.object.id, aco.link_name)

    # utilities
    def _rotate_tree(self, object_id):
        parent_id = self._get_parent_id(object_id)
        if parent_id != '':
            self._instance_props_dict[parent_id] = object_id
            self._rotate_tree(parent_id)

    def _set_target_link(self, aco, target_link, pose, touch_links):
        # Get a transform from the old attach link to the new one.
        T = tfs.concatenate_matrices(_pose_matrix(pose),
                                     tfs.inverse_matrix(
                                         _pose_matrix(aco.object.pose)))

        # Attach 'aco' to 'target_link' with 'pose'.
        aco.object.header.frame_id = target_link
        aco.object.pose            = pose
        aco.object.operation       = CollisionObject.ADD
        self._psi.attach_object(aco, target_link, touch_links)
        rospy.loginfo("(CollisionObjectManager) attached '%s' to '%s' with touch_links%s",
                      aco.object.id, aco.link_name, aco.touch_links)

        for child_id, child_aco in self._psi.get_attached_objects().items():
            if self._get_parent_id(child_id) == aco.object.id:
                self._set_target_link(child_aco, target_link,
                                      _transform_pose(T,
                                                      child_aco.object.pose),
                                      child_aco.touch_links)

    def _get_parent_id(self, object_id):
        return _get_object_id_and_subframe(
                   self._instance_props_dict[object_id].parent_link)[0]

    def _fix_target_link_and_pose(self, target_link, pose):
        tokens    = target_link.rsplit('/', 1)
        object_id = tokens[0]
        co        = self._get_object(object_id)
        if co is None:
            return target_link, pose
        subframe_name = tokens[1]
        return (co.header.frame_id,
                _pose_from_matrix(
                    tfs.concatenate_matrices(
                        _pose_matrix(co.pose),
                        _pose_matrix(_subframe_pose(co, subframe_name)),
                        _pose_matrix(pose))))

    def _generate_marker_id_list(self, n):
        marker_id_list = []
        for i in range(n):
            marker_id_list.append(self._marker_id_min)
            self._marker_id_min += 1
        return marker_id_list

    def _delete_markers_and_subframes(self, object_id):
        instance_props = self._instance_props_dict.get(object_id, None)
        if instance_props is None:
            rospy.logerr('(CollisionObjectManager) unknown object[%s]',
                         object_id)
            return
        for marker in instance_props.markers:
            marker.action = Marker.DELETE
            self._marker_pub.publish(marker)
        with self._lock:
            del self._instance_props_dict[object_id]
        rospy.loginfo("(CollisionObjectManager) removed '%s'", object_id)

    def _get_object(self, object_id):
        return self._psi.get_objects([object_id]) \
                        .get(object_id,
                             self._psi.get_objects([object_id]) \
                             .get(object_id, None))

    def _get_attached_object(self, object_id):
        return self._psi.get_attached_objects([object_id]).get(object_id, None)


#########################################################################
#  Entry point                                                          #
#########################################################################
if __name__ == '__main__':

  rospy.init_node('collision_object_manager', anonymous=True)

  server = CollisionObjectManager()
  rospy.spin()
