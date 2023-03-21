/******************************************************************************
 *      Title     : servo.h
 *      Project   : moveit_servo
 *      Created   : 1/11/2019
 *      Author    : Brian O'Neil, Andy Zelenak, Blake Anderson
 *
 *      Modified  : 10/3/2023
 *      Modifier  : Toshio Ueshiba
 *
 * BSD 3-Clause License
 *
 * Copyright (c) 2019, Los Alamos National Security, LLC
 * Copyright (c) 2023, National Institute of Advanced Industrial Science
 *		       and Technology(AIST)
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice,
 *   this list of conditions and the following disclaimer.
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
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
 * OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************/
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
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int8.h>
#include <std_srvs/Empty.h>
#include <tf2_eigen/tf2_eigen.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <ddynamic_reconfigure/ddynamic_reconfigure.h>
#include <aist_moveit_servo/DurationArray.h>

// aist_moveit_servo
#include <aist_moveit_servo/servo_parameters.h>
#include <aist_moveit_servo/collision_check.h>
#include <aist_moveit_servo/status_codes.h>
#include <aist_utility/butterworth_lpf.h>

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
class Servo
{
  public:
    using isometry3_t	= Eigen::Isometry3d;

  protected:
    using transform_t	= geometry_msgs::TransformStamped;
    using twist_t	= geometry_msgs::TwistStamped;
    using twist_cp	= geometry_msgs::TwistStampedConstPtr;
    using pose_t	= geometry_msgs::PoseStamped;
    using pose_cp	= geometry_msgs::PoseStampedConstPtr;
    using joint_jog_t	= control_msgs::JointJog;
    using joint_jog_cp	= control_msgs::JointJogConstPtr;
    using ddr_t		= ddynamic_reconfigure::DDynamicReconfigure;

  private:
    using planning_scene_monitor_p
			= planning_scene_monitor::PlanningSceneMonitorPtr;
    using joint_group_cp= const moveit::core::JointModelGroup*;
    using trajectory_t	= trajectory_msgs::JointTrajectory;
    using trajectory_cp	= trajectory_msgs::JointTrajectoryConstPtr;
    using multi_array_t	= std_msgs::Float64MultiArray;
    using flt64_t	= std_msgs::Float64;
    using flt64_cp	= std_msgs::Float64ConstPtr;
    using vector_t	= Eigen::VectorXd;
    using matrix_t	= Eigen::MatrixXd;
    using lpf_t		= aist_utility::ButterworthLPF<double>;

  public:
		Servo(ros::NodeHandle& nh, const std::string& logname)	;
		~Servo()						;

    ros::NodeHandle&
		nodeHandle()					const	;
    const std::string&
		logname()					const	;
    const ServoParameters&
		servoParameters()				const	;
    isometry3_t	getFrameTransform(const std::string& parent,
				  const std::string& child)	const	;
    isometry3_t	getFrameTransform(const std::string& frame)	const	;
    void	changeRobotLinkCommandFrame(const std::string& frame)	;

    DurationArray&
		durations()						;

  protected:
    void	start()							;
    void	stop()							;
    void	updateRobot()						;
    bool	publishTrajectory(const twist_t& twist_cmd,
				  const pose_t& ff_pose)		;
    template <class CMD>
    bool	publishTrajectory(const CMD& cmd, std::nullptr_t)	;
    StatusCode	servoStatus()					const	;
    void	resetServoStatus()					;

  private:
    uint	numJoints()					const	;
    joint_group_cp
		jointGroup()					const	;
    bool	isValid(const twist_t& cmd)			const	;
    bool	isValid(const joint_jog_t& cmd)			const	;

    template <class CMD>
    bool	publishTrajectory(const CMD& cmd,
				  const vector_t& positions)		;

    void	updateJoints()						;
    void	setTrajectory(const twist_t& cmd,
			      const vector_t& positions)		;
    void	setTrajectory(const joint_jog_t& cmd,
			      const vector_t& positions)		;

    vector_t	scaleCommand(const twist_t& cmd)		const	;
    vector_t	scaleCommand(const joint_jog_t& cmd)		const	;
    void	enforceVelLimits(vector_t& delta_theta)		const	;
    double	velocityScalingFactorForSingularity(
			const vector_t& commanded_velocity,
			const Eigen::JacobiSVD<matrix_t>& svd,
			const matrix_t& pseudo_inverse)			;
    void	applyVelocityScaling(vector_t& delta_theta,
				     double singularity_scale=1.0)	;

    void	convertDeltasToTrajectory(const vector_t& positions,
					  const vector_t& delta_theta)	;
    void	setPointsToTrajectory(const vector_t& positions,
				      const vector_t& delta_theta,
				      bool sudden=false)		;
    void	setZeroVelocitiesToTrajectory()				;

    bool	checkPositionLimits(const vector_t& positions,
				    const vector_t& delta_theta) const	;
    void	removeDimension(matrix_t& matrix, vector_t& delta_x,
				uint row_to_remove)		const	;


    void	initializeLowPassFilters(int half_order,
					 double cutoff_frequency)	;
    void	applyLowPassFilters(vector_t& positions)		;
    void	resetLowPassFilters()					;

    double	collisionVelocityScale()			const	;
    void	collisionVelocityScaleCB(const flt64_cp& velocity_scale);

    bool	changeDriftDimensionsCB(
			moveit_msgs::ChangeDriftDimensions::Request& req,
			moveit_msgs::ChangeDriftDimensions::Response& res);
    bool	changeControlDimensionsCB(
			moveit_msgs::ChangeControlDimensions::Request& req,
			moveit_msgs::ChangeControlDimensions::Response& res);

  // Servo status stuffs
    void	publishServoStatus()				const	;
    bool	resetServoStatusCB(std_srvs::Empty::Request&,
				   std_srvs::Empty::Response&)		;

  // Worst case stop time stuffs
    void	publishWorstCaseStopTime()			const	;

  private:
    ros::NodeHandle&			nh_;
    ros::NodeHandle			internal_nh_;

    const std::string			logname_;
    ServoParameters			parameters_;
    const planning_scene_monitor_p	planning_scene_monitor_;
    CollisionCheck			collision_checker_;

  // ROS
    const ros::Subscriber		collision_velocity_scale_sub_;
    const ros::Publisher		servo_status_pub_;
    const ros::Publisher		worst_case_stop_time_pub_;
    const ros::Publisher		outgoing_cmd_pub_;
    const ros::Publisher		outgoing_cmd_debug_pub_;
    const ros::Publisher		incoming_positions_debug_pub_;
    const ros::Publisher		durations_pub_;
    const ros::ServiceServer		drift_dimensions_srv_;
    const ros::ServiceServer		control_dimensions_srv_;
    const ros::ServiceServer		reset_servo_status_srv_;
    aist_moveit_servo::DurationArray	durations_;
    ddr_t				ddr_;

  // Incoming robot states
    moveit::core::RobotStatePtr		robot_state_;
    vector_t				actual_positions_;
    vector_t				actual_velocities_;
    vector_t				ff_positions_;

  // Track the number of cycles during which motion has not occurred.
  // Will avoid re-publishing zero velocities endlessly.
    int					invalid_command_count_;

  // Allow drift in [x, y, z, roll, pitch, yaw] in the command frame
    std::array<bool, 6>			drift_dimensions_;

  // Control [x, y, z, roll, pitch, yaw] in the command frame
    std::array<bool, 6>			control_dimensions_;

  // Output low-pass filters
    std::vector<lpf_t>			position_filters_;

  // Output command
    trajectory_t			joint_trajectory_;
    std::map<std::string, size_t>	joint_indices_;

  // Servo status and scaling factor of collision velocity
    StatusCode				servo_status_;
    flt64_cp				collision_velocity_scale_;

    mutable std::mutex			input_mtx_;
};

inline ros::NodeHandle&
Servo::nodeHandle() const
{
    return nh_;
}

inline const std::string&
Servo::logname() const
{
    return logname_;
}

inline const ServoParameters&
Servo::servoParameters() const
{
    return parameters_;
}

inline StatusCode
Servo::servoStatus() const
{
  // Guard servo_status_
    const std::lock_guard<std::mutex> lock(input_mtx_);

    return servo_status_;
}

inline Servo::isometry3_t
Servo::getFrameTransform(const std::string& parent,
			 const std::string& child) const
{
    return robot_state_->getGlobalLinkTransform(parent).inverse()
	 * robot_state_->getGlobalLinkTransform(child);
}

inline Servo::isometry3_t
Servo::getFrameTransform(const std::string& frame) const
{
    return getFrameTransform(parameters_.planning_frame, frame);
}

inline DurationArray&
Servo::durations()
{
    return durations_;
}

template <class CMD> bool
Servo::publishTrajectory(const CMD& cmd, std::nullptr_t)
{
    return publishTrajectory(cmd, actual_positions_);
}

//! Change the controlled link. Often, this is the end effector
/*!
  This must be a link on the robot since MoveIt tracks the transform (not tf)
*/
inline void
Servo::changeRobotLinkCommandFrame(const std::string& frame)
{
    parameters_.robot_link_command_frame = frame;
}

inline uint
Servo::numJoints() const
{
    return joint_trajectory_.joint_names.size();
}

inline Servo::joint_group_cp
Servo::jointGroup() const
{
    return robot_state_->getJointModelGroup(parameters_.move_group_name);
}

inline void
Servo::resetServoStatus()
{
  // Guard servo_status_
    const std::lock_guard<std::mutex> lock(input_mtx_);

    servo_status_ = StatusCode::NO_WARNING;
}

}  // namespace aist_moveit_servo
