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
 *  \file	pose_tracking_servo.h
 *  \brief	ROS pose tracker of aist_moveit_servo::PoseTracking type
 */
#pragma once

#include <actionlib/server/simple_action_server.h>
#include <ddynamic_reconfigure/ddynamic_reconfigure.h>
#include <control_toolbox/pid.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <aist_utility/butterworth_lpf.h>
#include <aist_moveit_servo/servo.h>
#include <aist_moveit_servo/status_codes.h>
#include <aist_moveit_servo/PoseTrackingAction.h>

// Conventions:
// Calculations are done in the planning_frame_ unless otherwise noted.

namespace aist_moveit_servo
{
/************************************************************************
*  class PoseTrackingServo						*
************************************************************************/
class PoseTrackingServo
{
  private:
    using server_t	 = actionlib::SimpleActionServer<PoseTrackingAction>;
    using goal_cp	 = boost::shared_ptr<const server_t::Goal>;
    using ddr_t		 = ddynamic_reconfigure::DDynamicReconfigure;
    using servo_status_t = aist_moveit_servo::StatusCode;
    using int8_cp	 = std_msgs::Int8ConstPtr;
    using twist_t	 = geometry_msgs::TwistStamped;
    using twist_cp	 = geometry_msgs::TwistStampedConstPtr;
    using pose_t	 = geometry_msgs::PoseStamped;
    using pose_cp	 = geometry_msgs::PoseStampedConstPtr;
    using raw_pose_t	 = geometry_msgs::Pose;
    using vector3_t	 = Eigen::Vector3d;
    using angle_axis_t	 = Eigen::AngleAxisd;
    using pid_t		 = control_toolbox::Pid;
    using lpf_t		 = aist_utility::ButterworthLPF<double, raw_pose_t>;

    struct PIDConfig
    {
      // Default values
	double dt	    = 0.001;
	double k_p	    = 1;
	double k_i	    = 0;
	double k_d	    = 0;
	double windup_limit = 0.1;
    };

  public:
		PoseTrackingServo(const ros::NodeHandle& nh)		;
		~PoseTrackingServo()					;

    void	run()							;
    void	tick()							;

  private:
    void	readROSParams()						;
    ros::Duration
		expectedCycleTime() const
		{
		    return ros::Duration(servo_.getParameters().publish_period);
		}

    void	servoStatusCB(const int8_cp& msg)	;
    void	targetPoseCB(const pose_cp& msg)			;
    void	goalCB()						;
    void	preemptCB()						;
    void	calculatePoseError(const raw_pose_t& offset,
				   vector3_t& positional_error,
				   angle_axis_t& angular_error)	const	;

    twist_cp	calculateTwistCommand(const vector3_t& positional_error,
				      const angle_axis_t& angular_error);
    void	stopMotion()						;
    void	doPostMotionReset()					;

  // Input low-pass filter stuffs
    void	updateInputLowPassFilter(int half_order,
					 double cutoff_frequency)	;

  // PID stuffs
    void	updatePositionPIDs(double PIDConfig::* field,
				   double value)			;
    void	updateOrientationPID(double PIDConfig::* field,
				     double value)			;
    void	updatePID(const PIDConfig& pid_config, pid_t& pid)	;

  // Target pose stuffs
    void	resetTargetPose()					;
    bool	haveRecentTargetPose(const ros::Duration& timeout) const;

  private:
    ros::NodeHandle		nh_;

    Servo			servo_;
    servo_status_t		servo_status_;

    ros::ServiceClient		reset_servo_status_;
    const ros::Subscriber	servo_status_sub_;
    const ros::Subscriber	target_pose_sub_;
    const ros::Publisher	twist_pub_;
    const ros::Publisher	predictive_pose_pub_;
    const ros::Publisher	target_pose_debug_pub_;
    const ros::Publisher	ee_pose_debug_pub_;
    DurationArray&		durations_;

  // Action server stuffs
    server_t			pose_tracking_srv_;
    goal_cp			current_goal_;

  // Dynamic reconfigure server
    ddr_t			ddr_;

  // Filters for input target pose
    int				input_low_pass_filter_half_order_;
    double			input_low_pass_filter_cutoff_frequency_;
    lpf_t			input_low_pass_filter_;

  // PIDs
    std::array<PIDConfig, 4>	pid_configs_;
    std::array<pid_t, 4>	pids_;

  // Servo inputs
    pose_t			target_pose_;
    mutable std::mutex		input_mtx_;
};

}	// namespace aist_moveit_servo
