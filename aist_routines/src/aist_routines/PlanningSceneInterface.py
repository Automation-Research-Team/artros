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

from collections                 import namedtuple
from tf                          import transformations as tfs
from geometry_msgs.msg           import (Point, Vector3, Quaternion, Pose,
                                         PoseStamped, TransformStamped)
from moveit_msgs.msg             import CollisionObject
from object_recognition_msgs.msg import ObjectType
from shape_msgs.msg              import (Mesh, MeshTriangle, Plane,
                                         SolidPrimitive)
from visualization_msgs.msg      import Marker
from std_msgs.msg                import ColorRGBA

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
    ObjectDescription = namedtuple('ObjectDescription',
                                   ['id', 'primitives', 'primitive_poses',
                                    'mesh_urls', 'meshes', 'mesh_poses',
                                    'planes', 'plane_poses',
                                    'subframe_names', 'subframe_poses'])

    def __init__(self, ns='', synchronous=True):
        super().__init__(ns, synchronous)
        self._object_descriptions = {}
        self._marker_ids          = {}
        self._marker_id_max       = 0
        self._pub = rospy.Publisher("collision_marker",
                                    Marker, queue_size=10)

    def load_objects(self, param_ns='~tool_descriptions'):
        PRIMITIVES = {'BOX':      SolidPrimitive.BOX,
                      'CYLINDER': SolidPrimitive.CYLINDER}

        for name, desc in rospy.get_param(param_ns, {}).items():
            pose = desc['pose']
            od = PlanningSceneInterface.ObjectDescription(name, [], [], [], [],
                                                          [], [], [], [], [])

            for primitive in desc.get('primitives', []):
                primitive_pose = primitive['pose']
                od.primitives.append(SolidPrimitive(
                    type=PRIMITIVES[primitive['type']],
                    dimensions=primitive['dimensions']))
                od.primitive_poses.append(
                    Pose(Point(*primitive_pose[0:3]),
                         Quaternion(*tfs.quaternion_from_euler(
                                        *np.radians(primitive_pose[3:6])))))

            for subframe_name, subframe_pose in desc.get('subframes',
                                                         {}).items():
                od.subframe_names.append(subframe_name)
                od.subframe_poses.append(
                    Pose(Point(*subframe_pose[0:3]),
                         Quaternion(*tfs.quaternion_from_euler(
                                        *np.radians(subframe_pose[3:6])))))

            for i, mesh in enumerate(desc.get('meshes', [])):
                od.mesh_urls.append(mesh['url'])
                mesh_pose = mesh['pose']
                od.meshes.append(self._load_mesh(mesh['url']))
                od.mesh_poses.append(
                    Pose(Point(*mesh_pose[0:3]),
                         Quaternion(*tfs.quaternion_from_euler(
                             *np.radians(mesh_pose[3:6])))))

            self._object_descriptions[name] = od
            rospy.loginfo('collision object[%s] loaded', name)

    def attach_object(self, name, pose, postfix='', use_mesh=False):
        # Create and attach a collision object.
        od = self._object_descriptions[name]
        co = CollisionObject()
        co.header = pose.header
        co.pose   = pose.pose
        co.id     = od.id + postfix
        if use_mesh:
            co.meshes     = od.meshes
            co.mesh_poses = od.mesh_poses
        else:
            co.primitives      = od.primitives
            co.primitive_poses = od.primitive_poses
        co.operation = CollisionObject.ADD
        super().attach_object(co, co.header.frame_id)

        # Publish shape of the collision object as visualization markers.
        self._marker_ids[co.id] = []
        for i, (mesh, mesh_pose) in enumerate(zip(od.meshes, od.mesh_poses)):
            marker = Marker()
            marker.header        = co.header
            marker.ns            = ''
            marker.id            = self._marker_id_max
            marker.type          = marker.MESH_RESOURCE
            marker.action        = Marker.ADD
            marker.pose          = self._compose_poses(co.pose, mesh_pose)
            marker.scale         = Vector3(0.001, 0.001, 0.001)
            marker.color         = ColorRGBA(0.0, 0.8, 0.5, 1.0)
            marker.lifetime      = rospy.Duration(0)
            marker.frame_locked  = False
            marker.mesh_resource = "file://" \
                                 + self._url_to_filepath(od.mesh_urls[i])
            self._pub.publish(marker)
            self._marker_ids[co.id].append(self._marker_id_max)
            self._marker_id_max += 1

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
                point = Point()
                point.x = vertex[0]*scale[0]
                point.y = vertex[1]*scale[1]
                point.z = vertex[2]*scale[2]
                mesh.vertices.append(point)
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

    def _compose_poses(self, pose0, pose1):
        T = tfs.concatenate_matrices(
                tfs.translation_matrix((pose0.position.x,
                                        pose0.position.y,
                                        pose0.position.z)),
                tfs.quaternion_matrix((pose0.orientation.x,
                                       pose0.orientation.y,
                                       pose0.orientation.z,
                                       pose0.orientation.w)),
                tfs.translation_matrix((pose1.position.x,
                                        pose1.position.y,
                                        pose1.position.z)),
                tfs.quaternion_matrix((pose1.orientation.x,
                                       pose1.orientation.y,
                                       pose1.orientation.z,
                                       pose1.orientation.w)))
        return Pose(Point(*tuple(tfs.translation_from_matrix(T))),
                    Quaternion(*tuple(tfs.quaternion_from_matrix(T))))
