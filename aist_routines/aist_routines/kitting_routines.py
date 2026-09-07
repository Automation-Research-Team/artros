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
import tf_transformations as tfs
import numpy as np

from math                        import pi, radians, degrees, cos, sin, sqrt
from geometry_msgs.msg           import Quaternion
from action_msgs.msg             import GoalStatus
from aist_graspability.client    import GraspabilityClient
from aist_graspability_msgs.msg  import Border, Point2D
from aist_utility.fileio         import filepath_from_url

from aist_tasks                  import AttemptBinTask
from aist_routines               import BaseRoutines

#************************************************************************
#  class KittingRoutines                                                *
#************************************************************************
class KittingRoutines(BaseRoutines):
    """Implements kitting routines for aist robot system."""

    def __init__(self, name: str):
        super().__init__(name)

        self._graspability_client = GraspabilityClient(self)
        self._attempt_bin         = AttemptBinTask(self)

    @property
    def bin_props(self):
        return self.settings['bin_props']

    @property
    def bin_ids(self):
        return list(self.bin_props.keys())

    @property
    def part_props(self):
        return self.settings['part_props']

    @property
    def part_ids(self):
        return list(self.part_props.keys())

    @property
    def borders(self):
        return self.settings['borders']

    @property
    def graspability_parameters(self):
        return self.settings['graspability_parameters']

    @property
    def fine_graspability_parameters(self):
        return self.settings['fine_graspability_parameters']

    # Interactive stuffs
    def do_cmds(self, dummy):
        """      Print command list."""
        super().do_cmds(dummy)
        print('=== Kitting commands ===')
        print('  s:  Search graspabilities with normal parameters')
        print('  sf: Search graspabilities with fine parameters')
        print('  a:  Attempt to pick and place')
        print('  A:  Repeat attempts to pick and place')

    def do_s(self, bin_id):
        """      s <bin_id>
        Search graspabilities in the specified bin with normal parameters."""
        self.search_bin(bin_id, False)

    def complete_s(self, text, line, ib, ie):
        return BaseRoutines._complete_default(text, line, self.bin_ids)

    def do_sf(self, bin_id):
        """      sf <bin_id>
        Search graspabilities in the specified bin with fine parameters."""
        self.search_bin(bin_id, True)

    def complete_sf(self, text, line, ib, ie):
        return BaseRoutines._complete_default(text, line, self.bin_ids)

    def do_a(self, bin_id):
        """      a <bin_id>
        Attempt to pick a part in the specified bin and place it."""
        self._recent_tasks[self._robot_name] = self._attempt_bin
        self._attempt_bin.send_goal(self._robot_name, bin_id, False, 5)

    def complete_a(self, text, line, ib, ie):
        return BaseRoutines._complete_default(text, line, self.bin_ids)

    def do_A(self, bin_id):
        """      A <bin_id>
        Attempt to pick all parts in the specified bin and place it."""
        self._recent_tasks[self._robot_name] = self._attempt_bin
        self._attempt_bin.send_goal(self._robot_name, bin_id, True, 5)

    def complete_A(self, text, line, ib, ie):
        return BaseRoutines._complete_default(text, line, self.bin_ids)

    # Commands
    def search_bin(self, bin_id, fine_parameters=False):
        if bin_id not in self.bin_ids:
            self.get_logger().error(
                'KittingRoutines.search_bin(): unknown bin_id[%s]' % bin_id)
            return GoalStatus.STATUS_ABORTED, None

        # Set parameters for searching graspabilities.
        bin_props = self.bin_props[bin_id]
        self._graspability_client.set_parameters(
            self.fine_graspability_parameters[bin_props['part_id']] \
            if fine_parameters else \
            self.graspability_parameters[bin_props['part_id']])

        # Set function for filtering graspabilities.
        if 'min_height' in bin_props and 'max_height' in bin_props:
            max_slant = 0.0 if fine_parameters else \
                        bin_props.get('max_slant', 45.0)
            self._graspability_client.set_graspability_filter(
                lambda graspabilities, \
                       target_frame=bin_props['name'], \
                       min_height=bin_props['min_height'], \
                       max_height=bin_props['max_height'], \
                       max_slant=max_slant:
                self._graspability_filter(graspabilities, target_frame,
                                          min_height, max_height, max_slant))
        else:
            self._graspability_client.set_graspability_filter(None)

        border     = self.borders[bin_props['border_id']]
        part_props = self.part_props[bin_props['part_id']]

        # Send goal first and then trigger camera frame.
        self._graspability_client.send_goal(
            Border(points=[Point2D(u=p[0], v=p[1]) for p in border]),
            self._grippers[part_props['gripper_name']].type,
            one_shot=True, timeout_sec=0.0)
        self.camera(part_props['camera_name']).trigger_frame()

        return self._graspability_client.wait()

    # Utilities
    def _graspability_filter(self, graspabilities, target_frame,
                             min_height, max_height, max_slant):
        def _pose_filter(pose, min_height, max_height, max_slant):
            def _normalize(x):
                return x / sqrt(np.dot(x, x))

            # Filter out poses whose height is not within the specified range.
            if pose.position.z < min_height or pose.position.z > max_height:
                return None

            T = tfs.quaternion_matrix((pose.orientation.x, pose.orientation.y,
                                       pose.orientation.z, pose.orientation.w))
            normal = T[0:3, 2]      # local Z-axis at the graspability point
            up     = np.array((0, 0, 1))

            # Cosine of angle between the graspability normal and up vector.
            a = np.dot(normal, up)
            b = cos(radians(max_slant))
            if a < b:
                p = sqrt((1.0 - b*b)/(1.0 - a*a))
                q = b - a*p
                R = np.identity(4, dtype=np.float32)
                R[0:3, 2] = p*normal + q*up                   # fixed Z-axis
                R[0:3, 1] = _normalize(np.cross(R[0:3, 2], T[0:3, 0]))
                R[0:3, 0] = np.cross(R[0:3, 1], R[0:3, 2])
                qR = tfs.quaternion_from_matrix(R)
                pose.orientation = Quaternion(x=qR[0], y=qR[1],
                                              z=qR[2], w=qR[3])
            return pose

        # We have to transform the graspabilitiy poses and contact points
        # to reference frame before moving because these are represented
        # w.r.t. camera frame which will change while moving in the case
        # of "eye on hand".
        graspabilities.contact_points = self.transform_points_to_target_frame(
                                            graspabilities.poses.header,
                                            graspabilities.contact_points,
                                            target_frame)
        graspabilities.poses          = self.transform_poses_to_target_frame(
                                            graspabilities.poses, (),
                                            target_frame)

        poses          = []
        gscores        = []
        contact_points = []
        for pose, gscore, contact_point \
            in zip(graspabilities.poses.poses, graspabilities.gscores,
                   graspabilities.contact_points):
            filtered_pose = _pose_filter(pose, min_height, max_height,
                                         max_slant)
            if filtered_pose is not None:
                poses.append(filtered_pose)
                gscores.append(gscore)
                contact_points.append(contact_point)
        graspabilities.poses.poses    = poses
        graspabilities.gscores        = gscores
        graspabilities.contact_points = contact_points
        return graspabilities
