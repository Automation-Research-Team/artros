/*******************************************************************************
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
/*      Title     : servo.cpp
 *      Project   : aist_moveit_servo
 *      Created   : 3/9/2017
 *      Author    : Brian O'Neil, Andy Zelenak, Blake Anderson, Toshio Ueshiba
 */
#include <aist_moveit_servo/make_shared_from_pool.h>
#include <aist_moveit_servo/servo.h>

static const std::string LOGNAME		= "servo_node";
static constexpr double	 ROBOT_STATE_WAIT_TIME	= 10.0;  // seconds

namespace aist_moveit_servo
{
/************************************************************************
*  global functions							*
************************************************************************/
planning_scene_monitor::PlanningSceneMonitorPtr
createPlanningSceneMonitor(const std::string& robot_description)
{
    using	namespace planning_scene_monitor;

    const auto	monitor = std::make_shared<PlanningSceneMonitor>(
				robot_description);
    if (!monitor->getPlanningScene())
    {
	ROS_ERROR_STREAM_NAMED(LOGNAME, "Failed to get PlanningSceneMonitor");
	exit(EXIT_FAILURE);
    }

    monitor->startSceneMonitor();
    monitor->startWorldGeometryMonitor(
	PlanningSceneMonitor::DEFAULT_COLLISION_OBJECT_TOPIC,
	PlanningSceneMonitor::DEFAULT_PLANNING_SCENE_WORLD_TOPIC,
	false /* skip octomap monitor */);
    monitor->startStateMonitor();

    ROS_INFO_STREAM_NAMED(LOGNAME, "PlanningSceneMonitor started");

    return monitor;
}

/************************************************************************
*  class Servo								*
************************************************************************/
Servo::Servo(const ros::NodeHandle& nh,
	     const planning_scene_monitor_p& planning_scene_monitor)
    :nh_(nh),
     parameters_(nh, LOGNAME),
     planning_scene_monitor_(planning_scene_monitor),
     servo_calcs_(nh_, parameters_, planning_scene_monitor_),
     collision_checker_(nh_, parameters_, planning_scene_monitor_)
{
  // Confirm the planning scene monitor is ready to be used
    if (!planning_scene_monitor_->getStateMonitor())
    {
	planning_scene_monitor_->startStateMonitor(parameters_.joint_topic);
    }
    planning_scene_monitor->getStateMonitor()->enableCopyDynamics(true);

    if (!planning_scene_monitor_->getStateMonitor()
	->waitForCompleteState(parameters_.move_group_name,
			       ROBOT_STATE_WAIT_TIME))
    {
	ROS_FATAL_NAMED(LOGNAME, "Timeout waiting for current state");
	exit(EXIT_FAILURE);
    }

  // Async spinner is needed to receive messages to wait for the robot state
  // to be complete
    ros::AsyncSpinner spinner(1);
    spinner.start();
}

Servo::~Servo()
{
    setPaused(true);
}

//! Start servo node
void
Servo::start()
{
    setPaused(false);

    servo_calcs_.start();		// Crunch the numbers in this timer
    if (parameters_.check_collisions)
	collision_checker_.start();	// Check collisions in this timer

}

//! Pause or unpause processing servo commands while keeping the timers alive
void
Servo::setPaused(bool paused)
{
    servo_calcs_.setPaused(paused);
    collision_checker_.setPaused(paused);
}

}  // namespace aist_moveit_servo
