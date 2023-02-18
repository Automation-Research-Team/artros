/*******************************************************************************
 *      Title     : servo_calcs.h
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
#include <std_msgs/Int8.h>
#include <std_srvs/Empty.h>
#include <tf2_eigen/tf2_eigen.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <ddynamic_reconfigure/ddynamic_reconfigure.h>
#include <aist_moveit_servo/DurationArray.h>

// aist_moveit_servo
#include <aist_moveit_servo/servo_parameters.h>
#include <aist_moveit_servo/status_codes.h>
#include <aist_utility/butterworth_lpf.h>

namespace aist_moveit_servo
{
/************************************************************************
*  class ServoCalcs							*
************************************************************************/
class ServoCalcs
{
  public:
    using isometry3_t	= Eigen::Isometry3d;

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
    using flt64_t	= std_msgs::Float64;
    using flt64_cp	= std_msgs::Float64ConstPtr;

    using vector_t	= Eigen::VectorXd;
    using matrix_t	= Eigen::MatrixXd;
    using lpf_t		= aist_utility::ButterworthLPF<double>;
    using ddr_t		= ddynamic_reconfigure::DDynamicReconfigure;

  public:
		ServoCalcs(const ros::NodeHandle& nh,
			   ServoParameters& parameters,
			   const planning_scene_monitor_p& monitor)	;
		~ServoCalcs()						;

    isometry3_t	getFrameTransform(const std::string& frame)	 const	;
    void	start()							;
    void	setPaused(bool paused)					;
    void	changeRobotLinkCommandFrame(
			const std::string& new_command_frame)		;

    aist_moveit_servo::DurationArray&
		durations()				{ return durations_; }

  // Give test access to private/protected methods
    friend class ServoFixture;

  private:
    isometry3_t	getFrameTransformUnlocked(const std::string& frame)
								const	;
    uint	num_joints()					const	;
    joint_group_cp
		joint_group()					const	;
    template <class MSG>
    bool	isStale(const MSG& msg)				const	;
    bool	isValid(const twist_t& msg)			const	;
    static bool	isValid(const joint_jog_t& msg)				;

    void	stop()							;

    void	mainCalcLoop()						;
    void	calculateSingleIteration()				;

    void	updateJoints()						;
    void	setCartesianServoTrajectory(twist_t& cmd)		;
    void	setJointServoTrajectory(const joint_jog_t& cmd)		;

    vector_t	scaleCartesianCommand(const twist_t& cmd)	const	;

    vector_t	scaleJointCommand(const joint_jog_t& cmd)	const	;
    void	enforceVelLimits(vector_t& delta_theta)		const	;
    double	velocityScalingFactorForSingularity(
			const vector_t& commanded_velocity,
			const Eigen::JacobiSVD<matrix_t>& svd,
			const matrix_t& pseudo_inverse)			;
    void	applyVelocityScaling(vector_t& delta_theta,
				     double singularity_scale=1.0)	;

    void	convertDeltasToTrajectory(const vector_t& delta_theta)	;
    void	setPointsToTrajectory(const vector_t& positions,
				      const vector_t& delta_theta,
				      bool sudden=false)		;
    void	zeroVelocitiesInTrajectory()				;

    bool	checkPositionLimits(const vector_t& positions,
				    const vector_t& delta_theta) const	;
    void	removeDimension(matrix_t& matrix, vector_t& delta_x,
				uint row_to_remove)		const	;


    void	initializeLowPassFilters(int half_order,
					 double cutoff_frequency)	;
    void	lowPassFilterPositions(vector_t& positions)		;

    void	resetLowPassFilters()					;

    void	twistCmdCB(const twist_cp& msg)				;
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
    ServoParameters&			parameters_;
    const planning_scene_monitor_p	planning_scene_monitor_;

  // ROS
    ros::NodeHandle			nh_;
    ros::NodeHandle			internal_nh_;
    const ros::Subscriber		twist_cmd_sub_;
    const ros::Subscriber		joint_cmd_sub_;
    const ros::Subscriber		collision_velocity_scale_sub_;
    ros::Publisher			status_pub_;
    ros::Publisher			worst_case_stop_time_pub_;
    ros::Publisher			outgoing_cmd_pub_;
    ros::Publisher			outgoing_cmd_debug_pub_;
    ros::Publisher			durations_pub_;
    const ros::ServiceServer		drift_dimensions_srv_;
    const ros::ServiceServer		control_dimensions_srv_;
    const ros::ServiceServer		reset_status_srv_;
    aist_moveit_servo::DurationArray	durations_;
    ddr_t				ddr_;

  // Track the number of cycles during which motion has not occurred.
  // Will avoid re-publishing zero velocities endlessly.
    int					invalid_command_count_;

  // Flag for staying inactive while there are no incoming commands
    bool				wait_for_servo_commands_;

  // Incoming command messages
    moveit::core::RobotStatePtr		robot_state_;
    ros::Time				robot_state_stamp_;
    vector_t				actual_positions_;
    vector_t				actual_velocities_;

  // Output command
    trajectory_t			joint_trajectory_;
    std::map<std::string, size_t>	joint_indices_;

  // Low-pass filters
    std::vector<lpf_t>			position_filters_;

  // Main tracking / result publisher loop
    std::thread				thread_;
    bool				stop_requested_;

  // Status
    StatusCode				status_;
    std::atomic<bool>			paused_;
    double				collision_velocity_scale_;

  // Allow drift in [x, y, z, roll, pitch, yaw] in the command frame
    std::array<bool, 6>			drift_dimensions_;

  // Control [x, y, z, roll, pitch, yaw] in the command frame
    std::array<bool, 6>			control_dimensions_;

  // input_mutex_ is used to protect the state below it
    mutable std::mutex			input_mutex_;
    twist_t				twist_cmd_;
    joint_jog_t				joint_cmd_;

  // input condition variable used for low latency mode
    std::condition_variable		input_cv_;
    bool				new_input_cmd_;
};

inline ServoCalcs::isometry3_t
ServoCalcs::getFrameTransform(const std::string& frame) const
{
    const std::lock_guard<std::mutex>	lock(input_mutex_);

    return getFrameTransformUnlocked(frame);
}

inline ServoCalcs::isometry3_t
ServoCalcs::getFrameTransformUnlocked(const std::string& frame) const
{
    return robot_state_->getGlobalLinkTransform(parameters_.planning_frame)
	  .inverse()
	 * robot_state_->getGlobalLinkTransform(frame);
}

inline uint
ServoCalcs::num_joints() const
{
    return joint_trajectory_.joint_names.size();
}

inline ServoCalcs::joint_group_cp
ServoCalcs::joint_group() const
{
    return robot_state_->getJointModelGroup(parameters_.move_group_name);
}

}  // namespace aist_moveit_servo
