/*******************************************************************************
 *      Title     : servo_calcs.h
 *      Project   : moveit_servo
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

// C++
#include <condition_variable>
#include <mutex>
#include <thread>
#include <atomic>

// ROS
#include <control_msgs/JointJog.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit_msgs/ChangeDriftDimensions.h>
#include <moveit_msgs/ChangeControlDimensions.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int8.h>
#include <std_srvs/Empty.h>
#include <tf2_eigen/tf2_eigen.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <ddynamic_reconfigure/ddynamic_reconfigure.h>
#include <aist_controllers/DurationArray.h>

// moveit_servo
#include <moveit_servo/servo_parameters.h>
#include <moveit_servo/status_codes.h>
#if defined(BUTTERWORTH)
#  include <aist_utility/butterworth_lpf.h>
#else
#  include <moveit_servo/low_pass_filter.h>
#endif

namespace moveit_servo
{
/************************************************************************
*  class ServoCalcs							*
************************************************************************/
class ServoCalcs
{
  private:
    using twist_t	     = geometry_msgs::TwistStamped;
    using twist_cp	     = geometry_msgs::TwistStampedConstPtr;
    using transform_t	     = geometry_msgs::TransformStamped;
    using joint_jog_t	     = control_msgs::JointJog;
    using joint_jog_cp	     = control_msgs::JointJogConstPtr;
    using trajectory_t	     = trajectory_msgs::JointTrajectory;
    using trajectory_point_t = trajectory_msgs::JointTrajectoryPoint;
    using joint_state_t	     = sensor_msgs::JointState;
    using f64_t		     = std_msgs::Float64;
    using f64_cp	     = std_msgs::Float64ConstPtr;
    
    using vector_t	     = Eigen::VectorXd;
    using matrix_t	     = Eigen::MatrixXd;
    using isometry3_t	     = Eigen::Isometry3d;
    
  public:
		ServoCalcs(ros::NodeHandle& nh, ServoParameters& parameters,
			   const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor);

		~ServoCalcs()						;

  /**
   * Get the MoveIt planning link transform.
   * The transform from the MoveIt planning frame to robot_link_command_frame
   *
   * @param transform the transform that will be calculated
   * @return true if a valid transform was available
   */
    bool	getCommandFrameTransform(isometry3_t& transform) const	;
    bool	getCommandFrameTransform(transform_t& transform) const	;

  /**
   * Get the End Effector link transform.
   * The transform from the MoveIt planning frame to EE link
   *
   * @param transform the transform that will be calculated
   * @return true if a valid transform was available
   */
    bool	getEEFrameTransform(isometry3_t& transform)	const	;
    bool	getEEFrameTransform(transform_t& transform)	const	;

    isometry3_t	getFrameTransform(const std::string& frame)	const	;

  /** \brief Start the timer where we do work and publish outputs */
    void	start()							;

  /** \brief Pause or unpause processing servo commands while keeping the timers alive */
    void	setPaused(bool paused)					;

  /** \brief Change the controlled link. Often, this is the end effector
   * This must be a link on the robot since MoveIt tracks the transform (not tf)
   */
    void	changeRobotLinkCommandFrame(
			const std::string& new_command_frame)		;

    aist_controllers::DurationArray&
		durations()				{ return durations_; }

  // Give test access to private/protected methods
    friend class ServoFixture;

  private:
  /** \brief Stop the currently running thread */
    void	stop()							;

  /** \brief Run the main calculation loop */
    void	mainCalcLoop()						;

  /** \brief Do calculations for a single iteration. Publish one outgoing command */
    void	calculateSingleIteration()				;

  /** \brief Parse the incoming joint msg for the joints of our MoveGroup */
    void	updateJoints()						;

  /** \brief Do servoing calculations for Cartesian twist commands. */
    bool	cartesianServoCalcs(twist_t& cmd,
				    trajectory_t& joint_trajectory)	;

  /** \brief Do servoing calculations for direct commands to a joint. */
    bool	jointServoCalcs(const joint_jog_t& cmd,
				trajectory_t& joint_trajectory)		;

  /** \brief  Scale the delta theta to match joint velocity/acceleration limits */
    void	enforceVelLimits(vector_t& delta_theta)		const	;

  /** \brief Possibly calculate a velocity scaling factor, due to proximity of
   * singularity and direction of motion
   */
    double	velocityScalingFactorForSingularity(
			const vector_t& commanded_velocity,
			const Eigen::JacobiSVD<matrix_t>& svd,
			const matrix_t& pseudo_inverse)			;

  /**
   * Slow motion down if close to singularity or collision.
   * @param delta_theta motion command, used in calculating new_joint_tray
   * @param singularity_scale tells how close we are to a singularity
   */
    void	applyVelocityScaling(vector_t& delta_theta,
				     double singularity_scale)		;

  /** \brief Convert joint deltas to an outgoing JointTrajectory command.
   * This happens for joint commands and Cartesian commands.
   */
    bool	convertDeltasToOutgoingCmd(const vector_t& delta_theta,
					   trajectory_t& joint_trajectory);

    bool	addJointIncrements(joint_state_t& output,
				   const vector_t& delta_theta)	const	;

  /** \brief Convert an incremental position command to joint velocity message */
    void	calculateJointVelocities(joint_state_t& joint_state,
					 const vector_t& delta_theta)	;

  /** \brief Compose the outgoing JointTrajectory message */
    void	composeJointTrajMessage(
			const joint_state_t& joint_state,
			trajectory_t& joint_trajectory)		const	;

  /** \brief Avoid overshooting joint limits */
    bool	enforcePositionLimits(joint_state_t& joint_state) const	;

  /** \brief Gazebo simulations have very strict message timestamp requirements.
   * Satisfy Gazebo by stuffing multiple messages into one.
   */
    void	insertRedundantPointsIntoTrajectory(
			trajectory_t& joint_trajectory, int count) const;

  /** \brief Suddenly halt for a joint limit or other critical issue.
   * Is handled differently for position vs. velocity control.
   */
    void	suddenHalt(trajectory_t& joint_trajectory)		;

  /**
   * Remove the Jacobian row and the delta-x element of one Cartesian dimension, to take advantage of task redundancy
   *
   * @param matrix The Jacobian matrix.
   * @param delta_x Vector of Cartesian delta commands, should be the same size as matrix.rows()
   * @param row_to_remove Dimension that will be allowed to drift, e.g. row_to_remove = 2 allows z-translation drift.
   */
    void	removeDimension(matrix_t& matrix, vector_t& delta_x,
				uint row_to_remove)		const	;
  /** \brief If incoming velocity commands are from a unitless joystick, scale them to physical units.
   * Also, multiply by timestep to calculate a position change.
   */
    vector_t	scaleCartesianCommand(const twist_t& command)	const	;

  /** \brief If incoming velocity commands are from a unitless joystick, scale them to physical units.
   * Also, multiply by timestep to calculate a position change.
   */
    vector_t	scaleJointCommand(const joint_jog_t& command)	const	;

  /** \brief Change order and/or cutoff of filters */
#if defined(BUTTERWORTH)
    void	initializeLowPassFilters(int half_order,
					 double cutoff_frequency)	;
#else
    void	initializeLowPassFilters(double coeff)			;
#endif

  /** \brief Smooth position commands with a lowpass filter */
    void	lowPassFilterPositions(joint_state_t& joint_state)	;

  /** \brief Set the filters to the specified values */
    void	resetLowPassFilters(const joint_state_t& joint_state)	;

  /* \brief Command callbacks */
    void	twistStampedCB(const twist_cp& msg)			;
    void	jointCmdCB(const joint_jog_cp& msg)			;
    void	collisionVelocityScaleCB(const f64_cp& msg)		;

  /**
   * Allow drift in certain dimensions. For example, may allow the wrist
   * to rotate freely.
   * This can help avoid singularities.
   *
   * @param request the service request
   * @param response the service response
   * @return true if the adjustment was made
   */
    bool	changeDriftDimensions(
			moveit_msgs::ChangeDriftDimensions::Request& req,
			moveit_msgs::ChangeDriftDimensions::Response& res);

  /** \brief Service callback for changing servoing dimensions (e.g. ignore rotation about X) */
    bool	changeControlDimensions(
			moveit_msgs::ChangeControlDimensions::Request& req,
			moveit_msgs::ChangeControlDimensions::Response& res);

  /** \brief Service callback to reset Servo status, e.g. so the arm can move again after a collision */
    bool	resetServoStatus(std_srvs::Empty::Request& req,
				 std_srvs::Empty::Response& res)	;

  private:
    ros::NodeHandle				nh_;

  // Parameters from yaml
    ServoParameters&				parameters_;

  // Pointer to the collision environment
    planning_scene_monitor::PlanningSceneMonitorPtr
						planning_scene_monitor_;

  // Track the number of cycles during which motion has not occurred.
  // Will avoid re-publishing zero velocities endlessly.
    int						zero_velocity_count_;

  // Flag for staying inactive while there are no incoming commands
    bool					wait_for_servo_commands_;

  // Flag saying if the filters were updated during the timer callback
    bool					updated_filters_;

    moveit::core::RobotStatePtr			current_state_;

  // Incoming command messages
    const moveit::core::JointModelGroup* const	joint_model_group_;


  // incoming_joint_state_ is the incoming message. It may contain passive
  // joints or other joints we don't care about.
  // (mutex protected below)
  // internal_joint_state_ is used in servo calculations. It shouldn't be
  // relied on to be accurate.
  // original_joint_state_ is the same as incoming_joint_state_
  // except it only contains the joints the servo node acts on.
    joint_state_t				internal_joint_state_,
						original_joint_state_;
    std::map<std::string, std::size_t>		joint_state_name_map_;

#if defined(BUTTERWORTH)
    std::vector<aist_utility::ButterworthLPF<double> >
						position_filters_;
#else
    std::vector<LowPassFilter>			position_filters_;
#endif

    trajectory_msgs::JointTrajectoryConstPtr	last_sent_command_;

  // ROS
    const ros::Subscriber			twist_stamped_sub_;
    const ros::Subscriber			joint_cmd_sub_;
    ros::Subscriber				collision_velocity_scale_sub_;
    ros::Publisher				status_pub_;
    ros::Publisher				worst_case_stop_time_pub_;
    ros::Publisher				outgoing_cmd_pub_;
    ros::Publisher				outgoing_cmd_debug_pub_;
    ros::Publisher				durations_pub_;
    const ros::ServiceServer			drift_dimensions_srv_;
    const ros::ServiceServer			control_dimensions_srv_;
    const ros::ServiceServer			reset_servo_status_srv_;
    aist_controllers::DurationArray		durations_;
    ddynamic_reconfigure::DDynamicReconfigure	ddr_;

  // Main tracking / result publisher loop
    std::thread			thread_;
    bool			stop_requested_;

  // Status
    StatusCode			status_;
    std::atomic<bool>		paused_;
    double			collision_velocity_scale_;

    const int			gazebo_redundant_message_count_;

    uint			num_joints_;

  // True -> allow drift in this dimension. In the command frame. [x, y, z, roll, pitch, yaw]
    std::array<bool, 6>		drift_dimensions_;

  // The dimesions to control. In the command frame. [x, y, z, roll, pitch, yaw]
    std::array<bool, 6>		control_dimensions_;

  // input_mutex_ is used to protect the state below it
    mutable std::mutex		input_mutex_;
    isometry3_t			tf_moveit_to_robot_cmd_frame_;
    isometry3_t			tf_moveit_to_ee_frame_;
    twist_t			twist_stamped_cmd_;
    joint_jog_t			joint_servo_cmd_;

  // input condition variable used for low latency mode
    std::condition_variable	input_cv_;
    bool			new_input_cmd_;
};
}  // namespace moveit_servo
