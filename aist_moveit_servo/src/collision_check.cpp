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

/*      Title     : collision_check.cpp
 *      Project   : aist_moveit_servo
 *      Created   : 1/11/2019
 *      Author    : Brian O'Neil, Andy Zelenak, Blake Anderson
 */
#include <aist_moveit_servo/collision_check.h>
#include <aist_moveit_servo/make_shared_from_pool.h>

static const char	LOGNAME[] = "collision_check";
static const double	MIN_RECOMMENDED_COLLISION_RATE = 10;
static constexpr double	EPSILON = 1e-6;                // For very small numeric comparisons
static constexpr size_t	ROS_LOG_THROTTLE_PERIOD = 30;  // Seconds to throttle logs inside loops

namespace aist_moveit_servo
{
/************************************************************************
*  class CollisionCheck							*
************************************************************************/
//! Constructor for the class that handles collision checking
/*!
  \param parameters		common settings of aist_moveit_servo
  \param planning_scene_monitor	PSM should have scene monitor and state monitor
				already started when passed into this class
*/
CollisionCheck::CollisionCheck(const ros::NodeHandle& nh,
			       const ServoParameters& parameters,
                               const planning_scene_monitor_p& planning_scene_monitor)
    :parameters_(parameters),
     planning_scene_monitor_(planning_scene_monitor),
     acm_(getLockedPlanningSceneRO()->getAllowedCollisionMatrix()),
     collision_check_type_(
	 parameters_.collision_check_type == "threshold_distance" ?
	 K_THRESHOLD_DISTANCE : K_STOP_DISTANCE),
     self_velocity_scale_coefficient_(
	 -log(0.001) / parameters.self_collision_proximity_threshold),
     scene_velocity_scale_coefficient_(
	 -log(0.001) / parameters.scene_collision_proximity_threshold),

     prev_collision_distance_(0),
     worst_case_stop_time_(std::numeric_limits<double>::max()),
     paused_(false),

     nh_(nh),
     internal_nh_(nh_, "internal"),
     worst_case_stop_time_sub_(
	 internal_nh_.subscribe("worst_case_stop_time", ROS_QUEUE_SIZE,
				&CollisionCheck::worstCaseStopTimeCB, this)),
     collision_velocity_scale_pub_(
	 internal_nh_.advertise<std_msgs::Float64>("collision_velocity_scale",
						   ROS_QUEUE_SIZE)),
     period_(1.0/parameters_.collision_check_rate),
     timer_()
{
  // Init collision request

    if (parameters_.collision_check_rate < MIN_RECOMMENDED_COLLISION_RATE)
	ROS_WARN_STREAM_THROTTLE_NAMED(
	    ROS_LOG_THROTTLE_PERIOD, LOGNAME,
	    "Collision check rate is low, increase it in yaml file if CPU allows");
}

/*
 *  private member functions
 */
//! Run one iteration of collision checking
void
CollisionCheck::run(const ros::TimerEvent& timer_event)
{
  // Log warning when the last loop duration was longer than the period
    if (timer_event.profile.last_duration.toSec() > period_.toSec())
	ROS_WARN_STREAM_THROTTLE_NAMED(
	    ROS_LOG_THROTTLE_PERIOD, LOGNAME,
	    "last_duration: "
	    << timer_event.profile.last_duration.toSec()
	    << " ("
	    << period_.toSec() << ")");
    
    if (paused_)
	return;

  // Update to the latest current state
    const auto	current_state = planning_scene_monitor_->getStateMonitor()
						       ->getCurrentState();
    current_state->updateCollisionBodyTransforms();

    bool	collision_detected	 = false;
    double	scene_collision_distance = 0;
    double	self_collision_distance	 = 0;

  // Do a thread-safe distance-based collision detection
    {  // Lock PlanningScene
	const auto	scene_ro = getLockedPlanningSceneRO();

      // Enable distance-based collision checking
      // and record the names of collision pairs
	collision_detection::CollisionRequest	collision_request;
	collision_request.group_name = parameters_.move_group_name;
	collision_request.distance   = true;
	collision_request.contacts   = true;

	collision_detection::CollisionResult	collision_result;
	collision_result.clear();
	scene_ro->getCollisionWorld()
		->checkRobotCollision(collision_request, collision_result,
				      *scene_ro->getCollisionRobot(),
				      *current_state, acm_);
	collision_detected	|= collision_result.collision;
	scene_collision_distance = collision_result.distance;

      // Self-collisions and scene collisions are checked separately
      // so different thresholds can be used
	collision_result.clear();
	scene_ro->getCollisionRobotUnpadded()
		->checkSelfCollision(collision_request, collision_result,
				     *current_state, acm_);
	collision_detected	|= collision_result.collision;
	self_collision_distance  = collision_result.distance;

	collision_result.print();
    }  // Unlock PlanningScene

  // Scale robot velocity according to collision proximity
  // and user-defined thresholds.
  // I scaled exponentially (cubic power) so velocity drops off quickly
  // after the threshold.
  // Proximity decreasing --> decelerate
    double	velocity_scale = 1;

  // If we're definitely in collision, stop immediately
    if (collision_detected)
    {
	velocity_scale = 0;
    }
  // If threshold distances were specified
    else if (collision_check_type_ == K_THRESHOLD_DISTANCE)
    {
      // If we are far from a collision, velocity_scale should be 1.
      // If we are very close to a collision, velocity_scale should be ~zero.
      // When scene_collision_proximity_threshold is breached,
      // start decelerating exponentially.
	if (scene_collision_distance <
	    parameters_.scene_collision_proximity_threshold)
	{
	  // velocity_scale = e ^ k * (collision_distance - threshold)
	  // k = - ln(0.001) / collision_proximity_threshold
	  // 
	  // velocity_scale should equal one
	  // when collision_distance is at collision_proximity_threshold.
	  // velocity_scale should equal 0.001
	  // when collision_distance is at zero.
	    velocity_scale =
		std::min(velocity_scale,
			 exp(scene_velocity_scale_coefficient_ *
			     (scene_collision_distance -
			      parameters_.scene_collision_proximity_threshold)));
	}

	if (self_collision_distance <
	    parameters_.self_collision_proximity_threshold)
	{
	    velocity_scale =
		std::min(velocity_scale,
			 exp(self_velocity_scale_coefficient_ *
			     (self_collision_distance -
			      parameters_.self_collision_proximity_threshold)));
	}
    }
  // Else, we stop based on worst-case stopping distance
    else
    {
      // Retrieve the worst-case time to stop,
      // based on current joint velocities

      // Calculate rate of change of distance to nearest collision
	const auto	collision_distance = std::min(scene_collision_distance,
						      self_collision_distance);
	const auto	derivative_of_collision_distance
			    = (collision_distance - prev_collision_distance_)
			    / period_.toSec();

	if (collision_distance <
	    parameters_.min_allowable_collision_distance &&
	    derivative_of_collision_distance <= 0)
	{
	    velocity_scale = 0;
	}
      // Only bother doing calculations
      // if we are moving toward the nearest collision
	else if (derivative_of_collision_distance < -EPSILON)
	{
	  // At the rate the collision distance is decreasing,
	  // how long until we collide?
	    const auto	est_time_to_collision
			    = fabs(collision_distance /
				   derivative_of_collision_distance);

	  // halt if we can't stop fast enough (including the safety factor)
	    if (est_time_to_collision <
		(parameters_.collision_distance_safety_factor *
		 worst_case_stop_time_))
	    {
		velocity_scale = 0;
	    }
	}

      // Update for the next iteration
	prev_collision_distance_ = collision_distance;
    }

  // publish message
    auto	msg = moveit::util::make_shared_from_pool<std_msgs::Float64>();
    msg->data = velocity_scale;
    collision_velocity_scale_pub_.publish(msg);
}

//! Callback for stopping time, from the thread that is aware of velocity and acceleration
void
CollisionCheck::worstCaseStopTimeCB(const std_msgs::Float64ConstPtr& msg)
{
    worst_case_stop_time_ = msg->data;
}

//! Get a read-only copy of the planning scene
planning_scene_monitor::LockedPlanningSceneRO
CollisionCheck::getLockedPlanningSceneRO() const
{
    return planning_scene_monitor::LockedPlanningSceneRO(
		planning_scene_monitor_);
}

}  // namespace aist_moveit_servo
