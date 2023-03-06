/*******************************************************************************
 *      Title     : servo.h
 *      Project   : aist_moveit_servo
 *      Created   : 3/9/2017
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

// System
#include <memory>

// MoveIt
#include <aist_moveit_servo/servo_parameters.h>
#include <aist_moveit_servo/servo_calcs.h>
#include <aist_moveit_servo/collision_check.h>

namespace aist_moveit_servo
{
/************************************************************************
*  global functions							*
************************************************************************/
planning_scene_monitor::PlanningSceneMonitorPtr
createPlanningSceneMonitor(const std::string& robot_description)	;

/************************************************************************
*  class Servo								*
************************************************************************/
//! Class Servo - Jacobian based robot control with collision avoidance.
class Servo
{
  public:
    using isometry3_t	= Eigen::Isometry3d;
    using vector_t	= Eigen::VectorXd;
    using pose_t	= geometry_msgs::PoseStamped;
    using planning_scene_monitor_p
			= planning_scene_monitor::PlanningSceneMonitorPtr;

  public:
		Servo(const ros::NodeHandle& nh,
		      const planning_scene_monitor_p& planning_scene_monitor);
		~Servo();

    void	start()							;
    void	setPaused(bool paused)					;

    isometry3_t	getCommandFrameTransform() const
		{
		    return getFrameTransform(parameters_
					     .robot_link_command_frame);
		}
    isometry3_t	getEEFrameTransform() const
		{
		    return getFrameTransform(parameters_.ee_frame_name);
		}
    isometry3_t	getFrameTransform(const std::string& frame) const
		{
		    return servo_calcs_.getFrameTransform(frame);
		}

    bool	getJointPositions(const pose_t& pose,
				  vector_t& joint_positions) const
		{
		    return servo_calcs_.getJointPositions(pose,
							  joint_positions);
		}

  //! Get the parameters used by servo node
    const ServoParameters&
		getParameters()	const
		{
		    return parameters_;
		}

  //! Change the controlled link. Often, this is the end effector
  /*!
    This must be a link on the robot since MoveIt tracks the transform (not tf)
  */
    void	changeRobotLinkCommandFrame(
			const std::string& new_command_frame)
		{
		    servo_calcs_.changeRobotLinkCommandFrame(
			new_command_frame);
		}

  // Give test access to private/protected methods
    friend class ServoFixture;

    DurationArray&
		durations()
		{
		    return servo_calcs_.durations();
		}

  private:
    ros::NodeHandle			nh_;
    ServoParameters			parameters_;
    const planning_scene_monitor_p	planning_scene_monitor_;
    ServoCalcs				servo_calcs_;
    CollisionCheck			collision_checker_;
};

}  // namespace aist_moveit_servo
