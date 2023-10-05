#!/usr/bin/python
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
import os, yaml
import rospy, rospkg

######################################################################
#  class URDFGenerator                                               #
######################################################################
class URDFGenerator(object):
    def __init__(self):
        super(URDFGenerator, self).__init__()
        self._root \
            = os.path.join(rospkg.RosPack().get_path('aist_description'),
                           'parts')

    def generate_urdfs(self):
        template_xacro = self._read_template('template.urdf.xacro')
        template_urdf  = self._read_template('template.urdf')

        for part_props in self._get_parts_list():
            self._generate_xacro(part_props, template_xacro)
            self._generate_urdf(part_props, template_urdf)

    def _read_template(self, filename):
        with open(os.path.join(self._root,
                               'urdf', 'templates', filename)) as f:
            return f.read()

    def _get_parts_list(self):
        with open(os.path.join(self._root, 'config', 'parts_list.yaml')) as f:
            return yaml.safe_load(f.read())['parts_list']

    def _get_subframes(self, part_name):
        try:
            with open(os.path.join(self._root, 'config', 'object_metadata',
                                   part_name + '.yaml')) as f:
                return yaml.safe_load(f.read())['subframes']
        except Exception:
            return []

    def _generate_xacro(self, part_props, template):
        part_type = part_props['type']
        macroname = 'assy_part_' + part_type[0:2]
        macro = template.replace('MACRONAME', macroname) \
                        .replace('CAD', part_props['cad'])

        subframes_urdf = ''
        for subframe in self._get_subframes(part_props['name']):
            link_name = subframe['name']
            xyzrpy    = subframe['pose_xyzrpy']
            new_joint = ''
            new_joint += '    <joint name=\"${prefix}' + macroname \
                       + '_' + link_name + '_joint\" type=\"fixed\"> \n'
            new_joint += '      <parent link=\"${prefix}' + macroname \
                       + '"/> \n'
            new_joint += '      <child link=\"${prefix}' + macroname \
                       + '_' + link_name + '\"/> \n'
            new_joint += '      <origin xyz=\"${' \
                       + str(xyzrpy[0]) + '} ${' \
                       + str(xyzrpy[1]) + '} ${' \
                       + str(xyzrpy[2]) + '}\" rpy=\"${' \
                       + str(xyzrpy[3]) + '} ${' \
                       + str(xyzrpy[4]) + '} ${' \
                       + str(xyzrpy[5]) + '}\"/> \n'
            new_joint += '    </joint> \n'
            new_joint += '    <link name=\"${prefix}' + macroname \
                       + '_' + link_name + '\"/> \n'
            new_joint += '    \n'
            subframes_urdf += new_joint

        macro = macro.replace('<!-- #SUBFRAMES -->', subframes_urdf)

        with open(os.path.join(self._root, 'urdf', part_type + '.urdf.xacro'),
                  'w+') as f:
            f.write(macro)
            print('%s.urdf.xacro generated' % part_type)

    def _generate_urdf(self, part_props, template):
        part_type = part_props['type']
        macroname = 'assy_part_' + part_type[0:2]
        macro = template.replace('MACRONAME', macroname) \
                        .replace('PARTNAME',  part_type)

        with open(os.path.join(self._root, 'urdf', part_type + '.urdf'),
                  'w+') as f:
            f.write(macro)
            print('%s.urdf generated' % part_type)

######################################################################
#  global functions                                                  #
######################################################################
if __name__ == '__main__':
    try:
        g = URDFGenerator()
        g.generate_urdfs()
    except Exception as e:
        print(e)
