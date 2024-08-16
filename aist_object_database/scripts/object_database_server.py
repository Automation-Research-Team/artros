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

from tf                import transformations as tfs
from geometry_msgs.msg import Point, Vector3, Quaternion, Pose
from shape_msgs.msg    import Mesh, MeshTriangle, Plane, SolidPrimitive
from std_msgs.msg      import ColorRGBA
from aist_msgs.msg     import MeshResource, ObjectProperties
from aist_msgs.srv     import (GetMeshResource, GetMeshResourceResponse,
                               GetObjectProperties, GetObjectPropertiesResponse)

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
#  class ObjectDatabaseServer                                           #
#########################################################################
class ObjectDatabaseServer(object):
    def __init__(self):
        super().__init__()

        PRIMITIVES = {'BOX':      SolidPrimitive.BOX,
                      'SPHERE':   SolidPrimitive.SPHERE,
                      'CYLINDER': SolidPrimitive.CYLINDER,
                      'CONE':     SolidPrimitive.CONE}

        self._obj_props = {}
        for name, desc in rospy.get_param('~object_descriptions', {}).items():

            obj_prop = ObjectProperties()

            for primitive in desc.get('primitives', []):
                primitive_pose = primitive['pose']
                obj_prop.primitives.append(SolidPrimitive(
                    type=PRIMITIVES[primitive['type']],
                    dimensions=primitive['dimensions']))
                obj_prop.primitive_poses.append(
                    Pose(Point(*primitive_pose[0:3]),
                         Quaternion(*tfs.quaternion_from_euler(
                                        *np.radians(primitive_pose[3:6])))))

            for subframe_name, subframe_pose in desc.get('subframes',
                                                         {}).items():
                obj_prop.subframe_names.append(subframe_name)
                obj_prop.subframe_poses.append(
                    Pose(Point(*subframe_pose[0:3]),
                         Quaternion(*tfs.quaternion_from_euler(
                                        *np.radians(subframe_pose[3:6])))))

            for mesh in desc.get('visual_meshes', []):
                obj_prop.visual_mesh_urls.append(mesh['url'])
                mesh_pose = mesh['pose']
                obj_prop.visual_mesh_poses.append(
                    Pose(Point(*mesh_pose[0:3]),
                         Quaternion(*tfs.quaternion_from_euler(
                             *np.radians(mesh_pose[3:6])))))
                obj_prop.visual_mesh_scales.append(Vector3(*mesh['scale']))
                obj_prop.visual_mesh_colors.append(ColorRGBA(*mesh['color']))

            for mesh in desc.get('collision_meshes', []):
                obj_prop.collision_meshes.append(
                    self._load_mesh(mesh['url'], mesh['scale']))
                mesh_pose = mesh['pose']
                obj_prop.collision_mesh_poses.append(
                    Pose(Point(*mesh_pose[0:3]),
                         Quaternion(*tfs.quaternion_from_euler(
                             *np.radians(mesh_pose[3:6])))))
                obj_prop.collision_mesh_scales.append(Vector3(*mesh['scale']))

            self._obj_props[name] = obj_prop
            rospy.loginfo('properties of object[%s] loaded', name)

        self._get_mesh_resource \
            = rospy.Service('~get_mesh_resource', GetMeshResource,
                            self._get_mesh_resource_cb)
        self._get_object_properties_list \
            = rospy.Service('~get_object_properties', GetObjectProperties,
                            self._get_object_properties_cb)

    # service callbacks
    def _get_mesh_resource_cb(self, req):
        res = GetMeshResourceResponse()
        res.mesh_resource = req.mesh_resource
        for obj_prop in self._obj_props.values():
            if req.mesh_resource in obj_prop.visual_mesh_urls:
                with open(self._url_to_filepath(req.mesh_resource), 'rb') as f:
                    res.data = f.read()
                rospy.loginfo('(ObjectDatabaseServer) Send response to GetMeshResource request for the mesh_url[%s]', req.mesh_resource)
                return res
        rospy.logerr('(ObjectDatabaseServer) Received GetMeshResource request with unknown mesh_url[%s]', req.mesh_resource)
        return res

    def _get_object_properties_cb(self, req):
        res = GetObjectPropertiesResponse()
        res.properties = self._obj_props[req.name]
        rospy.loginfo('(ObjectDatabaseServer) Send response to GetObjectProperties request for the object[%s]', req.name)
        return res

    # utilities
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

#########################################################################
#  Entry point                                                          #
#########################################################################
if __name__ == '__main__':

  rospy.init_node('object_database_server', anonymous=True)

  server = ObjectDatabaseServer()
  rospy.spin()
