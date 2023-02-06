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
    using planning_scene_monitor_p
			= planning_scene_monitor::PlanningSceneMonitorPtr;
    using joint_group_cp= const moveit::core::JointModelGroup*;
    using transform_t	= geometry_msgs::TransformStamped;
    using twist_t	= geometry_msgs::TwistStamped;
    using twist_cp	= geometry_msgs::TwistStampedConstPtr;
    using joint_jog_t	= control_msgs::JointJog;
    using joint_jog_cp	= control_msgs::JointJogConstPtr;
    using trajectory_t	= trajectory_msgs::JointTrajectory;
    using trajectory_cp	= trajectory_msgs::JointTrajectoryConstPtr;

    using trajectory_point_t = trajectory_msgs::JointTrajectoryPoint;
    
    using joint_state_t	= sensor_msgs::JointState;
    using flt64_t	= std_msgs::Float64;
    using flt64_cp	= std_msgs::Float64ConstPtr;
    
    using vector_t	= Eigen::VectorXd;
    using matrix_t	= Eigen::MatrixXd;
    using isometry3_t	= Eigen::Isometry3d;
#if defined(BUTTERWORTH)
    using lpf_t		= aist_utility::ButterworthLPF<double>;
#else
    using lpf_t		= LowPassFilter;
#endif
    using ddr_t		= ddynamic_reconfigure::DDynamicReconfigure;

  public:
		ServoCalcs(const ros::NodeHandle& nh,
			   ServoParameters& parameters,
			   const planning_scene_monitor_p& monitor)	;
		~ServoCalcs()						;

    bool	getCommandFrameTransform(isometry3_t& transform) const	;
    bool	getCommandFrameTransform(transform_t& transform) const	;
    bool	getEEFrameTransform(isometry3_t& transform)	 const	;
    bool	getEEFrameTransform(transform_t& transform)	 const	;
    isometry3_t	getFrameTransform(const std::string& frame)	 const	;

    void	start()							;
    void	setPaused(bool paused)					;
    void	changeRobotLinkCommandFrame(
			const std::string& new_command_frame)		;

    aist_controllers::DurationArray&
		durations()				{ return durations_; }

  // Give test access to private/protected methods
    friend class ServoFixture;

  private:
    isometry3_t	getFrameTransformUnlocked(const std::string& frame)
								const	;

    uint	num_joints()					const	;
    joint_group_cp
		joint_group()					const	;
    
    void	stop()							;

    void	mainCalcLoop()						;
    void	calculateSingleIteration()				;
    void	updateJoints()						;
    bool	cartesianServoCalcs(twist_t& cmd,
				    trajectory_t& joint_trajectory)	;
    bool	jointServoCalcs(const joint_jog_t& cmd,
				trajectory_t& joint_trajectory)		;

    vector_t	scaleCartesianCommand(const twist_t& command)	const	;
    vector_t	scaleJointCommand(const joint_jog_t& command)	const	;

    double	velocityScalingFactorForSingularity(
			const vector_t& commanded_velocity,
			const Eigen::JacobiSVD<matrix_t>& svd,
			const matrix_t& pseudo_inverse)			;
    void	applyVelocityScaling(vector_t& delta_theta,
				     double singularity_scale)		;
    bool	convertDeltasToOutgoingCmd(const vector_t& delta_theta,
					   trajectory_t& joint_trajectory);

    bool	addJointIncrements(joint_state_t& output,
				   const vector_t& delta_theta)	const	;

    void	calculateJointVelocities(joint_state_t& joint_state,
					 const vector_t& delta_theta)	;

    void	composeJointTrajMessage(
			const joint_state_t& joint_state,
			trajectory_t& joint_trajectory)		const	;
    bool	enforcePositionLimits(joint_state_t& joint_state) const	;

    void	insertRedundantPointsIntoTrajectory(
			trajectory_t& joint_trajectory, int count) const;
    void	suddenHalt(trajectory_t& joint_trajectory)		;

    void	removeDimension(matrix_t& matrix, vector_t& delta_x,
				uint row_to_remove)		const	;

#if defined(BUTTERWORTH)
    void	initializeLowPassFilters(int half_order,
					 double cutoff_frequency)	;
#else
    void	initializeLowPassFilters(double coeff)			;
#endif

    void	lowPassFilterPositions(joint_state_t& joint_state)	;

    void	resetLowPassFilters(const joint_state_t& joint_state)	;

    void	twistStampedCB(const twist_cp& msg)			;
    void	jointCmdCB(const joint_jog_cp& msg)			;
    void	collisionVelocityScaleCB(const flt64_cp& msg)		;

    bool	changeDriftDimensions(
			moveit_msgs::ChangeDriftDimensions::Request& req,
			moveit_msgs::ChangeDriftDimensions::Response& res);

    bool	changeControlDimensions(
			moveit_msgs::ChangeControlDimensions::Request& req,
			moveit_msgs::ChangeControlDimensions::Response& res);

  // Servo status stuffs
    void	publishStatus()					const	;
    bool	resetStatus(std_srvs::Empty::Request&,
			    std_srvs::Empty::Response&)			;

  // Worst case stop time stuffs
    void	publishWorstCaseStopTime()			const	;
    
  private:
    ros::NodeHandle				nh_;
    ros::NodeHandle				internal_nh_;

    ServoParameters&				parameters_;
    const planning_scene_monitor_p		planning_scene_monitor_;

  // Track the number of cycles during which motion has not occurred.
  // Will avoid re-publishing zero velocities endlessly.
    int						zero_velocity_count_;

  // Flag for staying inactive while there are no incoming commands
    bool					wait_for_servo_commands_;

  // Flag saying if the filters were updated during the timer callback
    bool					updated_filters_;

  // Incoming command messages
    moveit::core::RobotStatePtr			robot_state_;

  // incoming_joint_state_ is the incoming message. It may contain passive
  // joints or other joints we don't care about.
  // (mutex protected below)
  // joint_state_ is used in servo calculations. It shouldn't be
  // relied on to be accurate.
  // original_joint_state_ is the same as incoming_joint_state_
  // except it only contains the joints the servo node acts on.
    joint_state_t				joint_state_,
						original_joint_state_;
    std::map<std::string, std::size_t>		joint_indices_;

    std::vector<lpf_t>				position_filters_;

    trajectory_cp				last_sent_command_;

  // ROS
    const ros::Subscriber			twist_stamped_sub_;
    const ros::Subscriber			joint_cmd_sub_;
    const ros::Subscriber			collision_velocity_scale_sub_;
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

  // True -> allow drift in this dimension. In the command frame. [x, y, z, roll, pitch, yaw]
    std::array<bool, 6>		drift_dimensions_;

  // The dimesions to control. In the command frame. [x, y, z, roll, pitch, yaw]
    std::array<bool, 6>		control_dimensions_;

  // input_mutex_ is used to protect the state below it
    mutable std::mutex		input_mutex_;
    twist_t			twist_stamped_cmd_;
    joint_jog_t			joint_servo_cmd_;

  // input condition variable used for low latency mode
    std::condition_variable	input_cv_;
    bool			new_input_cmd_;
};

inline uint
ServoCalcs::num_joints() const
{
    return joint_state_.name.size();
}
    
inline ServoCalcs::joint_group_cp
ServoCalcs::joint_group() const
{
    return robot_state_->getJointModelGroup(parameters_.move_group_name);
}
}  // namespace moveit_servo
