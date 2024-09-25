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

def _decompose_link_name(link_name):
    tokens = link_name.rsplit('/', 1)
    if len(tokens) == 2:
        return tokens[0], tokens[1]
    else:
        return '', link_name

def _pose_matrix(pose):
    return tfs.concatenate_matrices(tfs.translation_matrix((pose.position.x,
                                                            pose.position.y,
                                                            pose.position.z)),
                                    tfs.quaternion_matrix((pose.orientation.x,
                                                           pose.orientation.y,
                                                           pose.orientation.z,
                                                           pose.orientation.w)))

def _pose_from_matrix(T):
    return Pose(Point(*tfs.translation_from_matrix(T)),
                Quaternion(*tfs.quaternion_from_matrix(T)))

def _subframe_pose(co, subframe_name):
    return co.subframe_poses[co.subframe_names.index(subframe_name)]

#########################################################################
#  class CollisionObjectManager                                         #
#########################################################################
class CollisionObjectManager(object):
    """Python interface for managing collision objects

    - Maintain tree structure of collision objects
    - Service server for responding to requests for mesh resource
    - Service server for responding to requests for managing collision objects
    - Publish subframes of collision objects to TF
    - Publish shape of collision objects to topic '~collision_marker'
      as visual markers
    """

    ObjectProperties = namedtuple('ObjectProperties',
                                  ['primitives', 'primitive_poses',
                                   'visual_mesh_urls', 'visual_mesh_poses',
                                   'visual_mesh_scales', 'visual_mesh_colors',
                                   'collision_meshes', 'collision_mesh_poses',
                                   'collision_mesh_scales',
                                   'subframe_names', 'subframe_poses'])
    class InstanceProperties(object):
        def __init__(self, type):
            self.type                = type
            self.subframe_transforms = []
            self.markers             = []

        @property
        def parent_link(self):
            return self.subframe_transforms[0].header.frame_id

    def __init__(self, ns='', synchronous=True):
        """Initialize collision object manager

        - Load object properties from parameter '~object_properties'
          for each type
        - Setup marker publisher '~collision_marker' as well as services
          '~get_mesh_resource' and '~manage_collision_object'
        """
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

    #
    # Callbacks
    #
    def _subframes_and_markers_cb(self, event):
        """Timer callback

        Publish subframes and visual markers periodically
        """
        with self._lock:
            for instance_props in self._instance_props_dict.values():
                for subframe_transform in instance_props.subframe_transforms:
                    subframe_transform.header.stamp = rospy.Time.now()
                    self._broadcaster.sendTransform(subframe_transform)
                for marker in instance_props.markers:
                    self._marker_pub.publish(marker)

    def _get_mesh_resource_cb(self, req):
        """Service callback for GetMeshResource

        Send response with binary mesh data according to the requested URL
        of mesh resource
        """
        res = GetMeshResourceResponse()
        res.mesh_resource = req.mesh_resource
        for obj_props in self._obj_props_dict.values():
            if req.mesh_resource in obj_props.visual_mesh_urls:
                with open(_url_to_filepath(req.mesh_resource), 'rb') as f:
                    res.data = f.read()
                rospy.loginfo('(ObjectDatabaseServer) Send response to GetMeshResource request for the mesh_url[%s]', req.mesh_resource)
                break
        else:
            rospy.logerr('(ObjectDatabaseServer) Received GetMeshResource request with unknown mesh_url[%s]', req.mesh_resource)
        return res

    def _manage_collision_object_cb(self, req):
        """Service callback for ManageCollisionObject

        Execute various operations on collision objects requested by clients
        """
        res = ManageCollisionObjectResponse(True, '')

        if req.op == ManageCollisionObjectRequest.CREATE_OBJECT:
            self._create_object(req.object_type, req.object_id,
                                req.frame_id, req.subframe,
                                req.pose)
        elif req.op == ManageCollisionObjectRequest.REMOVE_OBJECT:
            self._remove_object(req.object_id, req.frame_id)
        elif req.op == ManageCollisionObjectRequest.ATTACH_OBJECT:
            res.retval = self._attach_object(req.object_id,
                                                 req.frame_id,
                                                 req.subframe,
                                                 req.pose, req.touch_links)
        elif req.op == ManageCollisionObjectRequest.DETACH_OBJECT:
            self._detach_object(req.object_id, req.frame_id,
                                req.subframe, req.pose)
        elif req.op == ManageCollisionObjectRequest.APPEND_TOUCH_LINKS:
            self._append_touch_links(req.object_id, req.touch_links)
        elif req.op == ManageCollisionObjectRequest.REMOVE_TOUCH_LINKS:
            self._remove_touch_links(req.object_id, req.touch_links)
        elif req.op == ManageCollisionObjectRequest.GET_OBJECT_TYPE:
            instance_props = self._instance_props_dict.get(req.object_id)
            if instance_props is None:
                raise Exception('unknown object[%s]' % req.object_id)
            res.retval = instance_props.type
        elif req.op == ManageCollisionObjectRequest.GET_OBJECT_PARENT:
            instance_props = self._instance_props_dict.get(req.object_id)
            if instance_props is None:
                raise Exception('unknown object[%s]' % req.object_id)
            res.retval = instance_props.parent_link
        else:
            raise Exception('unknown operation[%d]' % req.op)

        # try:
        #     if req.op == ManageCollisionObjectRequest.CREATE_OBJECT:
        #         self._create_object(req.object_type, req.object_id,
        #                             req.frame_id, req.subframe,
        #                             req.pose)
        #     elif req.op == ManageCollisionObjectRequest.REMOVE_OBJECT:
        #         self._remove_object(req.object_id, req.frame_id)
        #     elif req.op == ManageCollisionObjectRequest.ATTACH_OBJECT:
        #         res.retval = self._attach_object(req.object_id,
        #                                          req.frame_id,
        #                                          req.subframe,
        #                                          req.pose, req.touch_links)
        #     elif req.op == ManageCollisionObjectRequest.DETACH_OBJECT:
        #         self._detach_object(req.object_id, req.frame_id,
        #                             req.subframe, req.pose)
        #     elif req.op == ManageCollisionObjectRequest.APPEND_TOUCH_LINKS:
        #         self._append_touch_links(req.object_id, req.touch_links)
        #     elif req.op == ManageCollisionObjectRequest.REMOVE_TOUCH_LINKS:
        #         self._remove_touch_links(req.object_id, req.touch_links)
        #     elif req.op == ManageCollisionObjectRequest.GET_OBJECT_TYPE:
        #         instance_props = self._instance_props_dict.get(req.object_id)
        #         if instance_props is None:
        #             raise Exception('unknown object[%s]' % req.object_id)
        #         res.retval = instance_props.type
        #     elif req.op == ManageCollisionObjectRequest.GET_OBJECT_PARENT:
        #         instance_props = self._instance_props_dict.get(req.object_id)
        #         if instance_props is None:
        #             raise Exception('unknown object[%s]' % req.object_id)
        #         res.retval = instance_props.parent_link
        #     else:
        #         raise Exception('unknown operation[%d]' % req.op)
        # except Exception as e:
        #     rospy.logerr('(CollisionObjectManager) %s', e)
        #     res.success = False

        return res

    #
    # Operations
    #
    def _create_object(self, object_type, object_id,
                       frame_id, subframe, pose):
        """Create a new collision object

        The created new collision object is not attached to any links
        and its pose is specified as that of subframe of the object
        with respect to the 'frame_id'.

        Args:
          object_type (str): type of object to be created
          object_id   (str): unique ID of object to identification
          frame_id    (str): reference frame for specifying pose of the object
          subframe    (str): subframe name with which the pose is specified
          pose (geometry_msgs/Pose): subframe pose w.r.t. 'frame_id'
        """
        obj_props = self._obj_props_dict.get(object_type)
        if obj_props is None:
            raise Exception('unknown object type[%s]' % req.object_type)

        # Setup a new collision object.
        co = CollisionObject()
        co.id = object_id
        if obj_props.collision_meshes != []:
            co.meshes     = obj_props.collision_meshes
            co.mesh_poses = obj_props.collision_mesh_poses
        else:
            co.primitives      = obj_props.primitives
            co.primitive_poses = obj_props.primitive_poses
        co.subframe_names = obj_props.subframe_names
        co.subframe_poses = obj_props.subframe_poses
        co.operation      = CollisionObject.ADD

        # If 'subframe' is not base_link of the object with 'object_id'
        # and/or 'frame_id' referes to any subframe of another object,
        # convert 'pose' to the one with respect to 'base_link'.
        frame_id, pose = self._get_parent_link_and_pose(co, frame_id,
                                                        subframe, pose)
        co.header.frame_id = frame_id
        co.pose            = pose

        # Create a new collision object.
        self._psi.add_object(co)

        # Create info for this object.
        instance_props = CollisionObjectManager.InstanceProperties(object_type)

        # Create subframe transforms.
        base_link = object_id + '/base_link'
        instance_props.subframe_transforms.append(
            TransformStamped(Header(frame_id=frame_id), base_link,
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
                                 object_id + '/' + subframe_name,
                                 Transform(Vector3(subframe_pose.position.x,
                                                   subframe_pose.position.y,
                                                   subframe_pose.position.z),
                                           Quaternion(
                                               subframe_pose.orientation.x,
                                               subframe_pose.orientation.y,
                                               subframe_pose.orientation.z,
                                               subframe_pose.orientation.w))))

        # Create new marker IDs if not exit for this object.
        if object_id not in self._marker_id_lists:
            self._marker_id_lists[object_id] \
              = self._generate_marker_id_list(len(obj_props.visual_mesh_urls))

        # Create markers for visualization.
        for mesh_url, mesh_pose, mesh_scale, mesh_color, marker_id \
            in zip(obj_props.visual_mesh_urls,   obj_props.visual_mesh_poses,
                   obj_props.visual_mesh_scales, obj_props.visual_mesh_colors,
                   self._marker_id_lists[object_id]):
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
            self._instance_props_dict[object_id] = instance_props

        rospy.loginfo("(CollisionObjectManager) created object '%s' of type[%s]",
                      co.id, object_type)

    def _remove_object(self, object_id, frame_id):
        if object_id != '':
            self._delete_markers_and_subframes(object_id)
        elif frame_id != '':
            object_id = None
            for aco_id, aco in self._psi.get_attached_objects().items():
                if aco.link_name == frame_id:
                    self._delete_markers_and_subframes(aco_id)
        else:
            object_id = None
            frame_id = None
            for co_id in self._psi.get_objects().keys():
                self._delete_markers_and_subframes(co_id)
            for aco_id in self._psi.get_attached_objects().keys():
                self._delete_markers_and_subframes(aco_id)
        self._psi.remove_attached_object(frame_id, object_id)
        self._psi.remove_world_object(object_id)

    def _attach_object(self, object_id, link, subframe, pose, touch_links):
        """Attach the object to the specified frame

        Attach the collision object 'object_id' to 'link'.

        Args:
          object_id   (str):  unique ID of the object to be attached
          link        (str):  link name to which the object attached with pose
                              specified by 'pose'
          subframe    (str):  subframe name with which the pose is specified
          pose (geometry_msgs/Pose): subframe pose w.r.t. 'link'
          touch_links (list): list of external links allowing collisions
                              against the object
        """
        aco = self._get_attached_object(object_id)
        if aco is None:
            co = self._get_object(object_id)
            if co is None:
                raise Exception("unknown collision object '%s'" % object_id)
            aco = AttachedCollisionObject(object=co)

        # If any subframe of the object other than 'base_link' is to be
        # attached to 'link', transform the given pose to that
        # with respect to 'base_link'.
        link, pose = self._get_parent_link_and_pose(aco.object,
                                                    link, subframe, pose)
        attach_link, attach_pose = self._get_attach_link_and_pose(link, pose)
        print('### parent_link=%s, my_id=%s' % (link, aco.object.id))

        # Make this object root of the tree attached to link.
        if attach_link != aco.object.header.frame_id:
            old_root_link = self._rotate_tree(aco)
        else:
            old_root_link = self._get_parent_link(object_id)

        # Keep the current parent link and update transform to the one
        # from 'base_link' to the new parent link.
        self._instance_props_dict[object_id].subframe_transforms[0] \
            = TransformStamped(Header(frame_id=link),
                               object_id + '/base_link',
                               Transform(Vector3(pose.position.x,
                                                 pose.position.y,
                                                 pose.position.z),
                                         Quaternion(pose.orientation.x,
                                                    pose.orientation.y,
                                                    pose.orientation.z,
                                                    pose.orientation.w)))

        # Set the new one to the specified object and its descendants.
        T = tfs.concatenate_matrices(_pose_matrix(attach_pose),
                                     tfs.inverse_matrix(
                                         _pose_matrix(aco.object.pose)))
        #aco.object.pose = Pose(Point(0, 0, 0), Quaternion(0, 0, 0, 1))
        self._attach_descendants(aco, attach_link, touch_links, T)

        return old_root_link

    def _detach_object(self, object_id, link, subframe, pose):
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

    #
    # Utilities
    #
    def _rotate_tree(self, aco):
        def _inverse_transform(transform):
            T = tfs.inverse_matrix(
                    tfs.concatenate_matrices(
                        tfs.translation_matrix(
                            (transform.transform.translation.x,
                             transform.transform.translation.y,
                             transform.transform.translation.z)),
                        tfs.quaternion_matrix(
                            (transform.transform.rotation.x,
                             transform.transform.rotation.y,
                             transform.transform.rotation.z,
                             transform.transform.rotation.w))))
            return TransformStamped(
                       Header(frame_id=transform.child_frame_id),
                       transform.header.frame_id,
                       Transform(Vector3(*tfs.translation_from_matrix(T)),
                                 Quaternion(*tfs.quaternion_from_matrix(T))))

        parent_aco = self._get_attached_object(self._get_parent_id(
                                                   aco.object.id))
        if parent_aco is None:
            return self._get_parent_link(aco.object.id)

        old_root_link = self._rotate_tree(parent_aco)
        self._instance_props_dict[parent_co.id].subframe_transforms[0] \
            = _inverse_transform(
                self._instance_props_dict[parent_aco.object.id] \
                    .subframe_transforms[0])
        tmp = self._instance_props_dict[parent_aco.object.id].subframe_transforms[0]
        print('### rotate_tree: %s <= %s' % (tmp.header.frame_id,
                                                 tmp.child_frame_id))
        return old_root_link

    def _attach_descendants(self, aco, link, touch_links, T):
        for child_aco in self._psi.get_attached_objects().values():
            print('### child_aco[%s]: parent_link=%s'
                  % (child_aco.object.id,
                     self._get_parent_link(child_aco.object.id)))
            if self._get_parent_id(child_aco.object.id) == aco.object.id:
                self._attach_descendants(child_aco,
                                         link, child_aco.touch_links, T)

        for child_co in self._psi.get_objects().values():
            print('### child_co[%s]: parent_link=%s'
                  % (child_co.id, self._get_parent_link(child_co.id)))
            if self._get_parent_id(child_co.id) == aco.object.id:
                child_co.header.frame_id = link
                child_co.pose = _pose_from_matrix(
                                    tfs.concatenate_matrices(
                                        T, _pose_matrix(child_co.pose)))
                child_aco = AttachedCollisionObject(object=child_co)
                self._psi.attach_object(child_aco, link,
                                        [aco.object.id + '/base_link'])
                rospy.loginfo("(CollisionObjectManager) attached '%s' to '%s' with touch_links%s",
                              child_aco.object.id, link,
                              [aco.object.id + '/base_link'])

        # Attach 'aco' to 'link' with 'pose'.
        aco.object.header.frame_id = link
        aco.object.pose = _pose_from_matrix(
                              tfs.concatenate_matrices(
                                  T, _pose_matrix(aco.object.pose)))
        self._psi.attach_object(aco, link, touch_links)
        rospy.loginfo("(CollisionObjectManager) attached '%s' to '%s' with touch_links%s",
                      aco.object.id, link, touch_links)

    def _get_object(self, object_id):
        return self._psi.get_objects([object_id]).get(object_id)

    def _get_attached_object(self, object_id):
        return self._psi.get_attached_objects([object_id]).get(object_id)

    def _get_parent_link(self, object_id):
        return self._instance_props_dict[object_id].parent_link

    def _get_parent_id(self, object_id):
        return _decompose_link_name(self._get_parent_link(object_id))[0]

    def _get_parent_link_and_pose(self, co, frame_id, subframe, pose):
        # Transform 'pose' w.r.t. 'subframe' to that w.r.t. base_link.
        if subframe != 'base_link':
            pose = _pose_from_matrix(
                       tfs.concatenate_matrices(
                           _pose_matrix(pose),
                           tfs.inverse_matrix(
                               _pose_matrix(_subframe_pose(co, subframe)))))

        # Separate 'frame_id' into object ID and subframe name.
        parent_id, parent_subframe = _decompose_link_name(frame_id)
        if parent_id == '':
            return frame_id, pose

        return (parent_id + '/base_link',
                _pose_from_matrix(
                   tfs.concatenate_matrices(
                       _pose_matrix(_subframe_pose(self._get_object(parent_id),
                                                   parent_subframe)),
                       _pose_matrix(pose))))

    def _get_attach_link_and_pose(self, frame_id, pose):
        co = self._get_object(_decompose_link_name(frame_id)[0])
        return (frame_id, pose) if co is None else \
               (co.header.frame_id,
                _pose_from_matrix(tfs.concatenate_matrices(
                    _pose_matrix(co.pose), _pose_matrix(pose))))

    def _generate_marker_id_list(self, n):
        marker_id_list = []
        for i in range(n):
            marker_id_list.append(self._marker_id_min)
            self._marker_id_min += 1
        return marker_id_list

    def _delete_markers_and_subframes(self, object_id):
        instance_props = self._instance_props_dict.get(object_id)
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

#########################################################################
#  Entry point                                                          #
#########################################################################
if __name__ == '__main__':

  rospy.init_node('collision_object_manager', anonymous=True)

  server = CollisionObjectManager()
  rospy.spin()
