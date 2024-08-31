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
import os, sys, copy, rospy, rospkg, ruamel.yaml, numpy as np
from numpy import pi
from tf    import transformations as tfs

YAML = ruamel.yaml.YAML()

######################################################################
#  global functions                                                  #
######################################################################
def eval_list(l):
    def _is_num(x):
        try:
            float(x)
        except ValueError:
            return False
        else:
            return True
    return [x if _is_num(x) else eval(x) for x in l]

def matrix_from_pose(pose):
    return tfs.translation_matrix(pose[0:3]) @ tfs.euler_matrix(*pose[3:6])

def to_degrees(pose):
    return flow_list(list(map(float, [*np.round(pose[0:3], 4),
                                      *np.round(np.degrees(pose[3:6]), 2)])))

def pose_from_base(mesh_pose, pose, rotconv=False):
    T = matrix_from_pose(mesh_pose) @ matrix_from_pose(pose)
    if rotconv:
        T = T @ tfs.euler_matrix(pi, pi/2, 0)
    return to_degrees([*tfs.translation_from_matrix(T),
                       *tfs.euler_from_matrix(T)])

def flow_list(l):
    ret = ruamel.yaml.comments.CommentedSeq(l)
    ret.fa.set_flow_style()  # fa -> format attribute
    return ret

def create_cops(part_info):
    part_id   = part_info['id']
    part_name = part_info['name']
    part_type = part_info['type']
    part_cad  = part_info['cad']
    part_desc = part_info['description']
    path = os.path.join(rospkg.RosPack().get_path('aist_description'),
                        'parts', 'config', 'o2ac', 'object_metadata',
                        part_name + '.yaml')
    with open(path) as file:
        part_props = YAML.load(file)

    mesh_pose  = eval_list(part_props.get('mesh_pose', [0, 0, 0, 0, 0, 0]))
    mesh_scale = [0.001, 0.001, 0.001]
    mesh_color = [0.2, 0.5, 0.8, 1.0]
    mesh_url   = 'package://aist_description/parts/meshes/' + part_cad
    cops = {'id':           part_id,
            'type':         part_type,
            'description':  part_desc,
            'visual_meshes':[{'url':   mesh_url,
                              'pose':  to_degrees(mesh_pose),
                              'scale': flow_list(mesh_scale),
                              'color': flow_list(mesh_color)}]}
    for name, prop in part_props.items():
        if name == 'collision_primitives':
            cops['primitives'] \
                = [{'type':       primitive['type'],
                    'dimensions': flow_list(primitive['dimensions']),
                    'pose':       pose_from_base(mesh_pose,
                                                 eval_list(primitive['pose']))}
                   for primitive in prop]
            print(cops['primitives'])
        elif name == 'subframes':
            cops['subframes'] \
                = {subframe['name']:
                   pose_from_base(mesh_pose,
                                  eval_list(subframe['pose_xyzrpy']), True)
                   for subframe in prop}
        elif name == 'grasp_points':
            cops['grasp_points'] \
                = {grasp_point['grasp_name']:
                   pose_from_base(mesh_pose,
                                  eval_list(grasp_point['pose_xyzrpy']), True)
                   for grasp_point in prop}
    return cops

if __name__ == '__main__':
    rospy.init_node('convert_o2ac_to_cop')

    cops_list  = {}
    for part_info in rospy.get_param('~parts_list')['parts_list']:
        try:
            cops_list[part_info['name']] = create_cops(part_info)
        except Exception as e:
            rospy.logerr(e)
    YAML.dump(cops_list, sys.stdout)

    path = os.path.join(rospkg.RosPack().get_path('aist_description'),
                        'parts', 'config', 'parts_properties.yaml')
    with open(path, 'w') as file:
        YAML.dump(cops_list, file)
