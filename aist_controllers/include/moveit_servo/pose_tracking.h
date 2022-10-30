/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, PickNik LLC
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of PickNik LLC nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/
/*
   Author: Andy Zelenak
   Desc: Servoing. Track a pose setpoint in real time.
*/

#pragma once

#include <atomic>
#include <boost/optional.hpp>
#include <control_toolbox/pid.h>
#include <moveit_servo/make_shared_from_pool.h>
#include <moveit_servo/servo.h>
#include <rosparam_shortcuts/rosparam_shortcuts.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <ddynamic_reconfigure/ddynamic_reconfigure.h>
#include <actionlib/server/simple_action_server.h>
#include <aist_controllers/PoseTrackingAction.h>

// Conventions:
// Calculations are done in the planning_frame_ unless otherwise noted.

namespace moveit_servo
{
struct PIDConfig
{
  // Default values
    double dt = 0.001;
    double k_p = 1;
    double k_i = 0;
    double k_d = 0;
    double windup_limit = 0.1;
};

enum class PoseTrackingStatusCode : int8_t
{
    INVALID = -1,
    SUCCESS = 0,
    NO_RECENT_TARGET_POSE = 1,
    NO_RECENT_END_EFFECTOR_POSE = 2,
    STOP_REQUESTED = 3
};

const std::unordered_map<PoseTrackingStatusCode, std::string>
POSE_TRACKING_STATUS_CODE_MAP(
    { { PoseTrackingStatusCode::INVALID, "Invalid" },
      { PoseTrackingStatusCode::SUCCESS, "Success" },
      { PoseTrackingStatusCode::NO_RECENT_TARGET_POSE, "No recent target pose" },
      { PoseTrackingStatusCode::NO_RECENT_END_EFFECTOR_POSE, "No recent end effector pose" },
      { PoseTrackingStatusCode::STOP_REQUESTED, "Stop requested" } });

/************************************************************************
*  class PoseTracking							*
************************************************************************/
/**
 * Class PoseTracking - subscribe to a target pose.
 * Servo toward the target pose.
 */
class PoseTracking
{
  private:
    using planning_scene_monitor_t
			    = planning_scene_monitor::PlanningSceneMonitor;
    using planning_scene_monitor_p
			    = planning_scene_monitor::PlanningSceneMonitorPtr;
    using server_t	    = actionlib::SimpleActionServer<
				aist_controllers::PoseTrackingAction>;
    using servo_status_t    = moveit_servo::StatusCode;
    using tracking_status_t = moveit_servo::PoseTrackingStatusCode;

  public:
  /** \brief Constructor. Loads ROS parameters under the given namespace. */
		PoseTracking()						;

  private:
    static planning_scene_monitor_p
		create_planning_scene_monitor(
		    const std::string& robot_description)		;

  /** \brief Load ROS parameters for controller settings. */
    void	readROSParams()						;

    void	goalCallback()						;

    void	preemptCallback()					;

    void	moveToPoseCallback(const ros::TimerEvent&)		;

  /** \brief Subscribe to the target pose on this topic */
    void	targetPoseCallback(
		    const geometry_msgs::PoseStampedConstPtr& msg)	;

  /** \brief A method for a different thread to stop motion and return early from control loop */
    void	stopMotion();

  /** \brief Re-initialize the target pose to an empty message. Can be used to reset motion between waypoints. */
    void	resetTargetPose()					;

  /** \brief Reset flags and PID controllers after a motion completes */
    void	doPostMotionReset()					;

  /** \brief Return true if a target pose has been received within timeout [seconds] */
    bool	haveRecentTargetPose(double timeout)		const	;

  /** \brief Return true if an end effector pose has been received within timeout [seconds] */
    bool	haveRecentEndEffectorPose(double timeout)	const	;

  /** \brief Check if XYZ, roll/pitch/yaw tolerances are satisfied */
    bool	satisfiesPoseTolerance(
		    const boost::array<double, 3>& positional_tolerance,
		    double angular_tolerance)			const	;

  /** \brief Update PID controller target positions & orientations */
    void	updateControllerSetpoints()				;

  /** \brief Update PID controller states (positions & orientations) */
    void	updateControllerStateMeasurements()			;

  /** \brief Use PID controllers to calculate a full spatial velocity toward a pose */
    geometry_msgs::TwistStampedConstPtr
		calculateTwistCommand(const geometry_msgs::Pose& offset);

  /** \brief Change PID parameters. Motion is stopped before the udpate */
    void	updatePIDConfig(double x_proportional_gain,
				double x_integral_gain,
				double x_derivative_gain,
				double y_proportional_gain,
				double y_integral_gain,
				double y_derivative_gain,
				double z_proportional_gain,
				double z_integral_gain,
				double z_derivative_gain,
				double angular_proportional_gain,
				double angular_integral_gain,
				double angular_derivative_gain)		;

  /** \brief Set parameter value to the specified field of position PID controllers for all directions */
    void	updatePositionPIDs(double PIDConfig::* field,
				   double value)			;

  /** \brief Initialize a PID controller and add it to vector of controllers */
    void	initializePID(const PIDConfig& pid_config,
			      std::vector<control_toolbox::Pid>& pid_vector);

  /** \brief Set parameter value to the specified field of orientation PID controller */
    void	updateOrientationPID(double PIDConfig::* field,
				     double value)			;

    void	getPIDErrors(double& x_error,
			     double& y_error,
			     double& z_error,
			     double& orientation_error)			;

  /**
   * Get the End Effector link transform.
   * The transform from the MoveIt planning frame to EE link
   *
   * @param transform the transform that will be calculated
   * @return true if a valid transform was available
   */
    bool	getEEFrameTransform(
		    geometry_msgs::TransformStamped& transform)		;

  public:
  // moveit_servo::Servo instance.
  // Public so we can access member functions like setPaused()
    std::unique_ptr<moveit_servo::Servo>	servo_;

  private:
    ros::NodeHandle				nh_;

    planning_scene_monitor_p			planning_scene_monitor_;
    robot_model::RobotModelConstPtr		robot_model_;
    const moveit::core::JointModelGroup*	joint_model_group_;
  // Joint group used for controlling the motions
    std::string					move_group_name_;

    ros::Rate					loop_rate_;

  // ROS interface to Servo
    ros::Publisher				twist_stamped_pub_;

  // PIDs
    std::vector<control_toolbox::Pid>		cartesian_position_pids_;
    std::vector<control_toolbox::Pid>		cartesian_orientation_pids_;
    PIDConfig					x_pid_config_,
						y_pid_config_,
						z_pid_config_,
						angular_pid_config_;

  // Transforms w.r.t. planning_frame_
    Eigen::Isometry3d				ee_frame_transform_;
    ros::Time					ee_frame_transform_stamp_;
    geometry_msgs::PoseStamped			target_pose_;
    mutable std::mutex				target_pose_mtx_;

  // For debugging
    ros::Publisher				ee_pose_pub_;

  // Subscribe to target pose
    ros::Subscriber				target_pose_sub_;

    tf2_ros::Buffer				transform_buffer_;
    tf2_ros::TransformListener			transform_listener_;

  // Expected frame name, for error checking and transforms
    std::string					planning_frame_;

  // Flag that a different thread has requested a stop.
    std::atomic<bool>				stop_requested_;

    boost::optional<double>			angular_error_;

  // Action server stuffs
    server_t					tracker_srv_;
    boost::shared_ptr<const server_t::Goal>	current_goal_;

  // Dynamic reconfigure server
    ddynamic_reconfigure::DDynamicReconfigure	ddr_;

  // Timer
    ros::Timer					timer_;
};

// using alias
using PoseTrackingPtr = std::shared_ptr<PoseTracking>;
}  // namespace moveit_servo
