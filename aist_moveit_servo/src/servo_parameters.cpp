/*******************************************************************************
 *      Title     : servo_parameters.cpp
 *      Project   : aist_moveit_servo
 *      Created   : 19/2/2023
 *      Author    : Toshio Ueshiba
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
#include <aist_moveit_servo/servo_parameters.h>
#include <rosparam_shortcuts/rosparam_shortcuts.h>

namespace aist_moveit_servo
{
/************************************************************************
*  class Servo::Parameters						*
************************************************************************/
ServoParameters::ServoParameters(const ros::NodeHandle& nh,
				 const std::string& logname)
{
    std::string	parameter_ns;
    auto	parameter_nh = (nh.getParam("parameter_ns", parameter_ns) ?
				ros::NodeHandle(nh, parameter_ns) : nh);
    std::size_t error = 0;

    error += !rosparam_shortcuts::get(logname, parameter_nh,
				      "publish_period", publish_period);
    error += !rosparam_shortcuts::get(logname, parameter_nh,
				      "collision_check_rate",
				      collision_check_rate);
    error += !rosparam_shortcuts::get(logname, parameter_nh,
				      "num_outgoing_halt_msgs_to_publish",
				      num_outgoing_halt_msgs_to_publish);
    error += !rosparam_shortcuts::get(logname, parameter_nh,
				      "scale/linear", linear_scale);
    error += !rosparam_shortcuts::get(logname, parameter_nh,
				      "scale/rotational", rotational_scale);
    error += !rosparam_shortcuts::get(logname, parameter_nh,
				      "scale/joint", joint_scale);
    error += !rosparam_shortcuts::get(logname, parameter_nh,
				      "low_pass_filter_half_order",
				      low_pass_filter_half_order);
    error += !rosparam_shortcuts::get(logname, parameter_nh,
				      "low_pass_filter_cutoff_frequency",
				      low_pass_filter_cutoff_frequency);
    error += !rosparam_shortcuts::get(logname, parameter_nh,
				      "joint_topic", joint_topic);
    error += !rosparam_shortcuts::get(logname, parameter_nh,
				      "command_in_type", command_in_type);
    error += !rosparam_shortcuts::get(logname, parameter_nh,
				      "cartesian_command_in_topic",
				      cartesian_command_in_topic);
    error += !rosparam_shortcuts::get(logname, parameter_nh,
				      "joint_command_in_topic",
				      joint_command_in_topic);
    error += !rosparam_shortcuts::get(logname, parameter_nh,
				      "robot_link_command_frame",
				      robot_link_command_frame);
    error += !rosparam_shortcuts::get(logname, parameter_nh,
				      "incoming_command_timeout",
				      incoming_command_timeout);
    error += !rosparam_shortcuts::get(logname, parameter_nh,
				      "lower_singularity_threshold",
				      lower_singularity_threshold);
    error += !rosparam_shortcuts::get(logname, parameter_nh,
				      "hard_stop_singularity_threshold",
				      hard_stop_singularity_threshold);
    error += !rosparam_shortcuts::get(logname, parameter_nh,
				      "move_group_name", move_group_name);
    error += !rosparam_shortcuts::get(logname, parameter_nh,
				      "planning_frame", planning_frame);
    error += !rosparam_shortcuts::get(logname, parameter_nh,
				      "ee_frame_name", ee_frame_name);
    error += !rosparam_shortcuts::get(logname, parameter_nh,
				      "use_gazebo", use_gazebo);
    error += !rosparam_shortcuts::get(logname, parameter_nh,
				      "revolute_joint_limit_margin",
				      revolute_joint_limit_margin);
    error += !rosparam_shortcuts::get(logname, parameter_nh,
				      "prismatic_joint_limit_margin",
				      prismatic_joint_limit_margin);
    error += !rosparam_shortcuts::get(logname, parameter_nh,
				      "command_out_topic", command_out_topic);
    error += !rosparam_shortcuts::get(logname, parameter_nh,
				      "command_out_type", command_out_type);
    error += !rosparam_shortcuts::get(logname, parameter_nh,
				      "publish_joint_positions",
				      publish_joint_positions);
    error += !rosparam_shortcuts::get(logname, parameter_nh,
				      "publish_joint_velocities",
				      publish_joint_velocities);
    error += !rosparam_shortcuts::get(logname, parameter_nh,
				      "publish_joint_accelerations",
				      publish_joint_accelerations);

  // Parameters for collision checking
    error += !rosparam_shortcuts::get(logname, parameter_nh,
				      "check_collisions", check_collisions);
    error += !rosparam_shortcuts::get(logname, parameter_nh,
				      "collision_check_type",
				      collision_check_type);
    error += !rosparam_shortcuts::get(logname, parameter_nh,
				      "self_collision_proximity_threshold",
				      self_collision_proximity_threshold);
    error += !rosparam_shortcuts::get(logname, parameter_nh,
				      "scene_collision_proximity_threshold",
				      scene_collision_proximity_threshold);
    error += !rosparam_shortcuts::get(logname, parameter_nh,
				      "collision_distance_safety_factor",
				      collision_distance_safety_factor);
    error += !rosparam_shortcuts::get(logname, parameter_nh,
				      "min_allowable_collision_distance",
				      min_allowable_collision_distance);
    error += !rosparam_shortcuts::get(logname, parameter_nh, "status_topic",
				      status_topic);
    error += !rosparam_shortcuts::get(logname, parameter_nh,
				      "low_latency_mode", low_latency_mode);

  // Parameters existence checking
    rosparam_shortcuts::shutdownIfError(logname, error);

  // Input checking
    error = 0;
    if (publish_period <= 0.)
    {
	ROS_WARN_NAMED(logname, "Parameter 'publish_period' should be "
		       "greater than zero. Check yaml file.");
	++error;
    }
    if (num_outgoing_halt_msgs_to_publish < 0)
    {
	ROS_WARN_NAMED(logname,
		       "Parameter 'num_outgoing_halt_msgs_to_publish' should be greater than zero. Check yaml file.");
	++error;
    }
    if (hard_stop_singularity_threshold <
	lower_singularity_threshold)
    {
	ROS_WARN_NAMED(logname, "Parameter 'hard_stop_singularity_threshold' "
		       "should be greater than 'lower_singularity_threshold.' "
		       "Check yaml file.");
	++error;
    }
    if ((hard_stop_singularity_threshold < 0.) || (lower_singularity_threshold < 0.))
    {
	ROS_WARN_NAMED(logname, "Parameters 'hard_stop_singularity_threshold' "
		       "and 'lower_singularity_threshold' should be "
		       "greater than zero. Check yaml file.");
	++error;
    }
    if (low_pass_filter_half_order <= 0)
    {
	ROS_WARN_NAMED(logname, "Parameter 'low_pass_filter_half_order' "
		       "should be positive. Check yaml fuke.");
	++error;
    }
    if (low_pass_filter_cutoff_frequency <= 0)
    {
	ROS_WARN_NAMED(logname, "Parameter 'low_pass_filter_cutoff_frequency' "
		       "should be positive. Check yaml fuke.");
	++error;
    }
    if (revolute_joint_limit_margin < 0.)
    {
	ROS_WARN_NAMED(logname, "Parameter 'revolute_joint_limit_margin' should be "
		       "greater than or equal to zero. Check yaml file.");
	++error;
    }
    if (prismatic_joint_limit_margin < 0.)
    {
	ROS_WARN_NAMED(logname, "Parameter 'prismatic_joint_limit_margin' should be "
		       "greater than or equal to zero. Check yaml file.");
	++error;
    }
    if (command_in_type != "unitless" &&
	command_in_type != "speed_units")
    {
	ROS_WARN_NAMED(logname, "command_in_type should be 'unitless' or "
		       "'speed_units'. Check yaml file.");
	++error;
    }
    if (command_out_type != "trajectory_msgs/JointTrajectory" &&
	command_out_type != "std_msgs/Float64MultiArray")
    {
	ROS_WARN_NAMED(logname, "Parameter command_out_type should be "
		       "'trajectory_msgs/JointTrajectory' or "
		       "'std_msgs/Float64MultiArray'. Check yaml file.");
	++error;
    }
    if (!publish_joint_positions && !publish_joint_velocities &&
	!publish_joint_accelerations)
    {
	ROS_WARN_NAMED(logname, "At least one of publish_joint_positions / "
		       "publish_joint_velocities / "
		       "publish_joint_accelerations must be true. Check "
		       "yaml file.");
	++error;
    }
    if ((command_out_type == "std_msgs/Float64MultiArray") &&
	publish_joint_positions &&
	publish_joint_velocities)
    {
	ROS_WARN_NAMED(logname, "When publishing a std_msgs/Float64MultiArray, "
		       "you must select positions OR velocities.");
	++error;
    }
  // Collision checking
    if (collision_check_type != "threshold_distance" &&
	collision_check_type != "stop_distance")
    {
	ROS_WARN_NAMED(logname, "collision_check_type must be 'threshold_distance' or 'stop_distance'");
	++error;
    }
    if (self_collision_proximity_threshold < 0.)
    {
	ROS_WARN_NAMED(logname, "Parameter 'self_collision_proximity_threshold' should be "
		       "greater than zero. Check yaml file.");
	++error;
    }
    if (scene_collision_proximity_threshold < 0.)
    {
	ROS_WARN_NAMED(logname, "Parameter 'scene_collision_proximity_threshold' should be "
		       "greater than zero. Check yaml file.");
	++error;
    }
    if (scene_collision_proximity_threshold <
	self_collision_proximity_threshold)
    {
	ROS_WARN_NAMED(logname, "Parameter 'self_collision_proximity_threshold' should probably be less "
		       "than or equal to 'scene_collision_proximity_threshold'. Check yaml file.");
    }
    if (collision_check_rate < 0)
    {
	ROS_WARN_NAMED(logname, "Parameter 'collision_check_rate' should be "
		       "greater than zero. Check yaml file.");
	++error;
    }
    if (collision_distance_safety_factor < 1)
    {
	ROS_WARN_NAMED(logname, "Parameter 'collision_distance_safety_factor' should be "
		       "greater than or equal to 1. Check yaml file.");
	++error;
    }
    if (min_allowable_collision_distance < 0)
    {
	ROS_WARN_NAMED(logname, "Parameter 'min_allowable_collision_distance' should be "
		       "greater than zero. Check yaml file.");
	++error;
    }

  // Parameter values checking
    rosparam_shortcuts::shutdownIfError(logname, error);
}

}	// namespace aist_moveit_servo
