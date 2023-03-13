/*******************************************************************************
 *      Title     : servo_parameters.h
 *      Project   : aist_moveit_servo
 *      Created   : 1/11/2019
 *      Author    : Brian O'Neil, Andy Zelenak, Blake Anderson
 *
 * BSD 3-Clause License
 *
 * Copyright (c) 2019, Los Alamos National Security, LLC
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

#pragma once

#include <ros/ros.h>

namespace aist_moveit_servo
{
// Size of queues used in ros pub/sub/service
constexpr size_t ROS_QUEUE_SIZE = 2;

// ROS params to be read. See the yaml file in /config for a description of each.
struct ServoParameters
{
    ServoParameters(const ros::NodeHandle& nh, const std::string& logname);

  // Gazebo
    bool	use_gazebo;

  // Properties of incoming commands
    std::string	command_in_type;

  // Scale parameters (only used if command_in_type == "unitless")
    double	linear_scale;
    double	rotational_scale;
    double	joint_scale;

  // Low-pass filter applied to output command
    int		low_pass_filter_half_order;
    double	low_pass_filter_cutoff_frequency;
    
  // Properties of outgoing commands
    std::string	command_out_type;
    double	publish_period;
    bool	low_latency_mode;
    bool	publish_joint_positions;
    bool	publish_joint_velocities;
    bool	publish_joint_accelerations;

  // MoveIt properties
    std::string	move_group_name;
    std::string	planning_frame;
    std::string	ee_frame_name;
    std::string	robot_link_command_frame;

  // Stopping behavior
    double	incoming_command_timeout;
    int		num_outgoing_halt_msgs_to_publish;

  // Configure handling of sigularities and joint limits
    double	lower_singularity_threshold;
    double	hard_stop_singularity_threshold;
    double	revolute_joint_limit_margin;
    double	prismatic_joint_limit_margin;

  // Topic names
    std::string	joint_topic;
    std::string	status_topic;
    std::string	command_out_topic;

  // Collision checking
    bool	check_collisions;
    std::string	collision_check_type;
    double	collision_check_rate;
    double	self_collision_proximity_threshold;
    double	scene_collision_proximity_threshold;
    double	collision_distance_safety_factor;
    double	min_allowable_collision_distance;
};

}  // namespace aist_moveit_servo
