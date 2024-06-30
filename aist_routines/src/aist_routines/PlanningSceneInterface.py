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
from tf                     import transformations as tfs
from geometry_msgs.msg      import (Point, Vector3, Quaternion, Pose,
                                    PoseStamped, TransformStamped)
from moveit_msgs.msg        import CollisionObject
from shape_msgs.msg         import Mesh, MeshTriangle, SolidPrimitive
from visualization_msgs.msg import Marker

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
    def __init__(self, ns='', synchronous=True):
        super().__init__(ns, synchronous)
        self._collision_objects = {}

    def load_objects(self, param_ns='~tool_descriptions'):
        PRIMITIVES = {'BOX':      SolidPrimitive.BOX,
                      'CYLINDER': SolidPrimitive.CYLINDER}

        for name, desc in rospy.get_param(param_ns, {}).items():
            pose = desc['pose']
            co = CollisionObject()
            co.header.frame_id = desc['frame_id']
            co.pose            = Pose(Point(*pose[0:3]),
                                      Quaternion(*tfs.quaternion_from_euler(
                                          *np.radians(pose[3:6]))))
            co.pose            = Pose(Point(0, 0, 0), Quaternion(0, 0, 0, 1))
            co.id              = name
            co.operation       = CollisionObject.ADD

            for primitive in desc.get('primitives', []):
                primitive_pose = primitive['pose']
                co.primitives.append(SolidPrimitive(
                    type=PRIMITIVES[primitive['type']],
                    dimensions=primitive['dimensions']))
                co.primitive_poses.append(
                    Pose(Point(*primitive_pose[0:3]),
                         Quaternion(*tfs.quaternion_from_euler(
                                        *np.radians(primitive_pose[3:6])))))

            for subframe_name, subframe_pose in desc.get('subframes',
                                                         {}).items():
                co.subframe_names.append(subframe_name)
                co.subframe_poses.append(
                    Pose(Point(*subframe_pose[0:3]),
                         Quaternion(*tfs.quaternion_from_euler(
                                        *np.radians(subframe_pose[3:6])))))

            for mesh in desc.get('meshes', []):
                mesh_pose = mesh['pose']
                co.meshes.append(self._load_mesh(mesh['url']))
                co.mesh_poses.append(
                    Pose(Point(*mesh_pose[0:3]),
                         Quaternion(*tfs.quaternion_from_euler(
                             *np.radians(mesh_pose[3:6])))))

            self._collision_objects[name] = co
            rospy.loginfo('collision object[%s] loaded', name)

    def attach_object(self, name, pose, postfix='', use_mesh=True):
        co = copy.copy(self._collision_objects[name])
        co.header = pose.header
        co.pose   = pose.pose
        co.id     = co.id + postfix
        if use_mesh:
            co.primitives      = []
            co.primitive_poses = []
        else:
            co.meshes     = []
            co.mesh_poses = []
        super().attach_object(co, co.header.frame_id)

    def create_marker(self, object_name, pose,
                      color=None, lifetime=0, frame_locked=False):
        co = self.get_collision_object(object_name)

        marker = Marker()
        marker.id            = object_name
        marker.header        = pose.header
        marker.pose          = pose.pose
        marker.frame_locked  = frame_locked
        marker.lifetime      = rospy.Duration(lifetime)
        marker.type          = marker.MESH_RESOURCE
        marker.mesh_resource = "file://" + self._url_to_filepath(
                                               self.get_mesh_url(object_name))
        marker.scale         = Vector3(0.001, 0.001, 0.001)
        if color:
            marker.color = color
        else:
            marker.color.a = 1.0
            marker.color.g = .8
            marker.color.b = .5
            marker.action = marker.ADD
        return marker

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
