/*******************************************************************************
 * Title     : collision_check.h
 * Project   : aist_moveit_servo
 * Created   : 1/11/2019
 * Author    : Brian O'Neil, Andy Zelenak, Blake Anderson
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

#include <moveit/collision_detection/collision_common.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>

#include <aist_moveit_servo/servo_parameters.h>

namespace aist_moveit_servo
{
enum CollisionCheckType
{
  K_THRESHOLD_DISTANCE = 1,
  K_STOP_DISTANCE = 2
};

/************************************************************************
*  class CollisionCheck							*
************************************************************************/
class CollisionCheck
{
  private:
    using planning_scene_monitor_p
		= planning_scene_monitor::PlanningSceneMonitorPtr;
    
  public:
		CollisionCheck(
		    ros::NodeHandle& nh, const ServoParameters& parameters,
		    const planning_scene_monitor_p& planning_scene_monitor);
		~CollisionCheck()
		{
		    timer_.stop();
		}

  //! Start the Timer that regulates collision check rate
    void	start()
		{
		    timer_ = nh_.createTimer(period_,
					     &CollisionCheck::run, this);
		}

  //! Pause or unpause processing servo commands while keeping the timers alive
    void	setPaused(bool paused)
		{
		    paused_ = paused;
		}

  private:
    void	run(const ros::TimerEvent& timer_event)			;
    void	worstCaseStopTimeCB(const std_msgs::Float64ConstPtr& msg);
    planning_scene_monitor::LockedPlanningSceneRO
		getLockedPlanningSceneRO()			const	;

  private:
  // Parameters from yaml
    const ServoParameters&			parameters_;

  // Pointer to the collision environment
    const planning_scene_monitor_p		planning_scene_monitor_;

  // Robot state and collision matrix from planning scene
    std::shared_ptr<moveit::core::RobotState>	current_state_;
    collision_detection::AllowedCollisionMatrix	acm_;

  // Scale robot velocity according to collision proximity and user-defined thresholds.
  // I scaled exponentially (cubic power) so velocity drops off quickly after the threshold.
  // Proximity decreasing --> decelerate
    const CollisionCheckType		collision_check_type_;
    double				velocity_scale_;
    double				self_collision_distance_;
    double				scene_collision_distance_;
    bool				collision_detected_;
    bool				paused_;

  // Variables for stop-distance-based collision checking
    double				current_collision_distance_;
    double				derivative_of_collision_distance_;
    double				prev_collision_distance_;
    double				est_time_to_collision_;
    double				safety_factor_;
    double				worst_case_stop_time_;
    
    const double			self_velocity_scale_coefficient_;
    const double			scene_velocity_scale_coefficient_;

  // collision request
    collision_detection::CollisionRequest	collision_request_;
    collision_detection::CollisionResult	collision_result_;

  // ROS
    const ros::NodeHandle		nh_;
    ros::NodeHandle			nh_internal_;
    const ros::Subscriber		worst_case_stop_time_sub_;
    const ros::Publisher		collision_velocity_scale_pub_;
    const ros::Duration			period_;
    ros::Timer				timer_;
};
}  // namespace aist_moveit_servo
