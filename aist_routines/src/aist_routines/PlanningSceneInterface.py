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
import moveit_commander
import threading

from collections                 import namedtuple
from tf                          import transformations as tfs
from geometry_msgs.msg           import (Point, Vector3, Quaternion,
                                         Pose, Transform,
                                         PoseStamped, TransformStamped)
from moveit_msgs.msg             import CollisionObject
from object_recognition_msgs.msg import ObjectType
from shape_msgs.msg              import (Mesh, MeshTriangle, Plane,
                                         SolidPrimitive)
from visualization_msgs.msg      import Marker
from std_msgs.msg                import Header, ColorRGBA
from tf2_ros                     import TransformBroadcaster

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
#  class PlanningSceneInterface                                         #
#########################################################################
class PlanningSceneInterface(moveit_commander.PlanningSceneInterface):
    CollisionObjectProps = namedtuple('CollisionObjectProps',
                                      ['primitives', 'primitive_poses',
                                       'visual_mesh_urls',
                                       'visual_mesh_poses',
                                       'visual_mesh_scales',
                                       'collision_meshes',
                                       'collision_mesh_poses',
                                       'collision_mesh_scales',
                                       'planes', 'plane_poses',
                                       'subframe_names', 'subframe_poses'])
    MarkerProps = namedtuple('MarkerProps', ['ids', 'markers', 'link'])

    def __init__(self, ns='', synchronous=True):
        super().__init__(ns, synchronous)
        self._marker_id_max          = 0
        self._collision_object_props = {}
        self._subframe_transforms    = {}
        self._markers                = {}
        self._marker_pub             = rospy.Publisher("collision_marker",
                                                       Marker, queue_size=10)
        self._lock                   = threading.Lock()
        th = threading.Thread(target=self._subframes_and_markers_thread)
        th.daemon = True
        th.start()

    def load_object_descriptions(self, object_descriptions):
        PRIMITIVES = {'BOX':      SolidPrimitive.BOX,
                      'SPHERE':   SolidPrimitive.SPHERE,
                      'CYLINDER': SolidPrimitive.CYLINDER,
                      'CONE':     SolidPrimitive.CONE}

        for name, desc in object_descriptions.items():
            cop = PlanningSceneInterface.CollisionObjectProps([], [],
                                                              [], [], [],
                                                              [], [], [],
                                                              [], [], [], [])

            for primitive in desc.get('primitives', []):
                primitive_pose = primitive['pose']
                cop.primitives.append(SolidPrimitive(
                    type=PRIMITIVES[primitive['type']],
                    dimensions=primitive['dimensions']))
                cop.primitive_poses.append(
                    Pose(Point(*primitive_pose[0:3]),
                         Quaternion(*tfs.quaternion_from_euler(
                                        *np.radians(primitive_pose[3:6])))))

            for subframe_name, subframe_pose in desc.get('subframes',
                                                         {}).items():
                cop.subframe_names.append(subframe_name)
                cop.subframe_poses.append(
                    Pose(Point(*subframe_pose[0:3]),
                         Quaternion(*tfs.quaternion_from_euler(
                                        *np.radians(subframe_pose[3:6])))))

            for mesh in desc.get('visual_meshes', []):
                cop.visual_mesh_urls.append(mesh['url'])
                mesh_pose = mesh['pose']
                cop.visual_mesh_poses.append(
                    Pose(Point(*mesh_pose[0:3]),
                         Quaternion(*tfs.quaternion_from_euler(
                             *np.radians(mesh_pose[3:6])))))
                cop.visual_mesh_scales.append(Vector3(*mesh['scale']))

            for mesh in desc.get('collision_meshes', []):
                cop.collision_meshes.append(
                    self._load_mesh(mesh['url'], mesh['scale']))
                mesh_pose = mesh['pose']
                cop.collision_mesh_poses.append(
                    Pose(Point(*mesh_pose[0:3]),
                         Quaternion(*tfs.quaternion_from_euler(
                             *np.radians(mesh_pose[3:6])))))
                cop.collision_mesh_scales.append(Vector3(*mesh['scale']))

            self._collision_object_props[name] = cop
            rospy.loginfo('collision object[%s] loaded', name)

    def attach_object(self, name, pose, id=None, touch_links=None):
        # Create and attach a collision object.
        cop = self._collision_object_props[name]
        co  = CollisionObject()
        co.header.frame_id = pose.header.frame_id
        co.pose            = pose.pose
        co.id              = name if id is None else id
        if cop.collision_meshes != []:
            co.meshes     = cop.collision_meshes
            co.mesh_poses = cop.collision_mesh_poses
        else:
            co.primitives      = cop.primitives
            co.primitive_poses = cop.primitive_poses
        co.operation = CollisionObject.ADD
        super().attach_object(co, co.header.frame_id, touch_links)

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
        for subframe_name, subframe_pose in zip(cop.subframe_names,
                                                cop.subframe_poses):
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
        for mesh_url, mesh_pose, mesh_scale in zip(cop.visual_mesh_urls,
                                                   cop.visual_mesh_poses,
                                                   cop.visual_mesh_scales):
            marker = Marker()
            marker.header.frame_id = base_link
            marker.ns              = ''
            marker.id              = self._marker_id_max
            marker.type            = marker.MESH_RESOURCE
            marker.action          = Marker.ADD
            marker.pose            = mesh_pose
            marker.scale           = mesh_scale
            marker.color           = ColorRGBA(0.0, 0.8, 0.5, 1.0)
            marker.lifetime        = rospy.Duration(0)
            marker.frame_locked    = False
            marker.mesh_resource   = "file://" + self._url_to_filepath(mesh_url)
            markers.append(marker)
            self._marker_id_max += 1

        # Keep created subframe transforms and markers.
        with self._lock:
            self._subframe_transforms[co.id] = subframe_transforms
            self._markers[co.id] = markers

    def add_touch_links_to_attached_object(self, id, touch_links):
        co = CollisionObject()
        co.id = id
        super().attach_object(id, touch_links=touch_links)

    def move_attached_object(self, id, pose, touch_links=None):
        if not self.get_attached_objects([id]):
            rospy.logerr('PlanningSceneInterface.move_attached_object(): unknown attached object[%s]', id)
            return
        co = CollisionObject()
        co.header.frame_id = pose.header.frame_id
        co.pose            = pose.pose
        co.id              = id
        co.operation       = CollisionObject.MOVE
        super().attach_object(co, co.header.frame_id, touch_links)

        # Publish visualization markers again.
        for marker in self._markers[id]:
            self._marker_pub.publish(marker)

        # Replace the transform from the object to the attached link.
        with self._lock:
            self._subframe_transforms[id][0] \
                = TransformStamped(co.header, co.id,
                                   Transform(Vector3(pose.pose.position.x,
                                                     pose.pose.position.y,
                                                     pose.pose.position.z),
                                             Quaternion(
                                                 pose.pose.orientation.x,
                                                 pose.pose.orientation.y,
                                                 pose.pose.orientation.z,
                                                 pose.pose.orientation.w)))

    def remove_attached_object(self, link=None, id=None):
        if id is not None:
            markers = self._markers.get(id, None)
            if markers is None:
                rospy.logerr('PlanningSceneInterface.remove_attached_object(): unknown attached object[%s]', id)
                return
            for marker in markers:
                self._delete_marker(marker.id)
            with self._lock:
                del self._subframe_transforms[id]
                del self._markers[id]
        elif link is not None:
            for id, aco in self.get_attached_objects().items():
                if aco.link_name == link:
                    for marker in self._markers[id]:
                        self._delete_marker(marker.id)
                    with self._lock:
                        del self._subframe_transforms[id]
                        del self._markers[id]
        else:
            for markers in self._markers.values():
                for marker in markers:
                    self._delete_marker(marker.id)
            self._marker_id_max = 0
            with self._lock:
                self._subframe_transforms.clear()
                self._markers.clear()
        super().remove_attached_object(link, id)
        self.remove_world_object(id)

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
            rospy.logerr('Failed to load mesh: %s', e)
            return None

    def _url_to_filepath(self, url):
        tokens = url.split('/')
        if len(tokens) < 2 or tokens[0] != 'package:' or tokens[1] != '':
            raise('Illegal URL: ' + url)
        return os.path.join(rospkg.RosPack().get_path(tokens[2]), *tokens[3:])
