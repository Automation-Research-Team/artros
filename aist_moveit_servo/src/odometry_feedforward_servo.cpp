/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2023, National Institute of Industrial Scienece
 *  and Technology (AIST)
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
 *  \file	odometry_feedforward_servo.cpp
 *  \brief	ROS pose tracking servo with odometry feedforward
 *  \author	Toshio UESHIBA
 */
#include <nav_msgs/Odometry.h>
#include <aist_moveit_servo/pose_tracking_servo.h>

namespace aist_moveit_servo
{
class OdometryFeedForward
{
  public:
    using pose_t	= geometry_msgs::PoseStamped;

  private:
    using odometry_cp	= nav_msgs::OdometryConstPtr;
    using twist_t	= geometry_msgs::TwistStamped;
    using vector3_t	= Eigen::Vector3d;
    using angle_axis_t	= Eigen::AngleAxisd;

  public:
		OdometryFeedForward(const Servo& servo)			;

    void	resetInput()						;
    bool	haveRecentInput(const ros::Duration& timeout)	const	;
    pose_t	ff_pose(const pose_t& target_pose,
			const ros::Duration& dt)		const	;

  private:
    twist_t	getTwist()					const	;
    void	odometryCB(const odometry_cp& odometry)			;

  private:
    const Servo&		servo_;
    const ros::Subscriber	odometry_sub_;
    odometry_cp			odometry_;
    mutable std::mutex		odometry_mtx_;
};

OdometryFeedForward::OdometryFeedForward(const Servo& servo)
    :servo_(servo),
     odometry_sub_(servo_.nodeHandle().subscribe(
		       "/odom", 1, &OdometryFeedForward::odometryCB, this)),
     odometry_(nullptr),
     odometry_mtx_()
{
}

void
OdometryFeedForward::resetInput()
{
    const std::lock_guard<std::mutex>	lock(odometry_mtx_);

    odometry_ = nullptr;
}

bool
OdometryFeedForward::haveRecentInput(const ros::Duration& timeout) const
{
    const std::lock_guard<std::mutex>	lock(odometry_mtx_);

    return (odometry_ != nullptr &&
	    ros::Time::now() - odometry_->header.stamp < timeout);
}

OdometryFeedForward::pose_t
OdometryFeedForward::ff_pose(const pose_t& target_pose,
			     const ros::Duration& dt) const
{
  // Get transform to base frame.
    const auto	twist = getTwist();
    const auto	T     = servo_.getFrameTransform(twist.header.frame_id,
						 target_pose.header.frame_id);

  // Get transform produced from twist.
    const vector3_t	angular(twist.twist.angular.x,
				twist.twist.angular.y,
				twist.twist.angular.z);
    const auto		d = dt.toSec();
    Servo::isometry3_t	dT;
    dT.linear()	     = angle_axis_t(d * angular.norm(), angular.normalized())
		      .toRotationMatrix();
    dT.translation() = vector3_t(d * twist.twist.linear.x,
				 d * twist.twist.linear.y,
				 d * twist.twist.linear.z);

  // Correct target pose by twist.
    auto	dS = tf2::eigenToTransform(T.inverse() * dT.inverse() * T);
    dS.header	      = target_pose.header;
    dS.child_frame_id = target_pose.header.frame_id;
    pose_t	pose;
    tf2::doTransform(target_pose, pose, dS);

    return  pose;
}

OdometryFeedForward::twist_t
OdometryFeedForward::getTwist() const
{
    const std::lock_guard<std::mutex> lock(odometry_mtx_);

    twist_t	twist;
    twist.header.stamp	  = odometry_->header.stamp;
    twist.header.frame_id = odometry_->child_frame_id;
    twist.twist		  = odometry_->twist.twist;

    return twist;
}

void
OdometryFeedForward::odometryCB(const odometry_cp& odometry)
{
    const std::lock_guard<std::mutex> lock(odometry_mtx_);

    odometry_ = odometry;
}

}	// namespace aist_moveit_servo

/************************************************************************
*  main function							*
************************************************************************/
int
main(int argc, char* argv[])
{
    using namespace aist_moveit_servo;

    const std::string	logname("odometry_feedforward_servo");

    ros::init(argc, argv, logname);

    ros::NodeHandle				nh("~");
    PoseTrackingServo<OdometryFeedForward>	servo(nh, logname);
    servo.run();

    return 0;
}
