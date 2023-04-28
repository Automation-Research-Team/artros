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
class OdometryFeedForwardServo
    : public PoseTrackingServo<OdometryFeedForwardServo>
{
  private:
    using super		= PoseTrackingServo<OdometryFeedForwardServo>;
    using odometry_cp	= nav_msgs::OdometryConstPtr;
    using twist_t	= geometry_msgs::TwistStamped;
    using vector3_t	= Eigen::Vector3d;
    using angle_axis_t	= Eigen::AngleAxisd;

  public:
		OdometryFeedForwardServo(ros::NodeHandle& nh,
					 const std::string& logname)	;

    void	resetFeedForwardInput()					;
    bool	haveRecentFeedForwardInput(const ros::Duration& timeout)
								const	;
    pose_t	ff_pose(const pose_t& target_pose,
    			const ros::Duration& dt)		const	;
    // std::nullptr_t
    // 		ff_pose(const pose_t& target_pose,
    // 			const ros::Duration& dt)		const	;

  private:
    twist_t	getTwist()					const	;
    void	odometryCB(const odometry_cp& odometry)			;

  private:
    const ros::Subscriber	odometry_sub_;
    odometry_cp			odometry_;
    mutable std::mutex		odometry_mtx_;
};

OdometryFeedForwardServo::OdometryFeedForwardServo(ros::NodeHandle& nh,
						   const std::string& logname)
    :super(nh, logname),
     odometry_sub_(nh.subscribe("/odom", 1,
				&OdometryFeedForwardServo::odometryCB, this)),
     odometry_(nullptr),
     odometry_mtx_()
{
}

void
OdometryFeedForwardServo::resetFeedForwardInput()
{
    const std::lock_guard<std::mutex>	lock(odometry_mtx_);

    odometry_ = nullptr;
}

bool
OdometryFeedForwardServo::haveRecentFeedForwardInput(const ros::Duration&
						     timeout) const
{
    const std::lock_guard<std::mutex>	lock(odometry_mtx_);

    return (odometry_ != nullptr &&
	    ros::Time::now() - odometry_->header.stamp < timeout);
}

OdometryFeedForwardServo::pose_t
  //std::nullptr_t
OdometryFeedForwardServo::ff_pose(const pose_t& target_pose,
				  const ros::Duration& dt) const
{
#if 1
  // Convert the subscribed velocity to the frame describing the target pose.
    auto	twist = getTwist();
    auto	T = tf2::eigenToTransform(getFrameTransform(
					      target_pose.header.frame_id,
					      twist.header.frame_id));
    auto	velocity = twist.twist.linear;
    T.header	     = target_pose.header;
    T.child_frame_id = twist.header.frame_id;
    tf2::doTransform(velocity, velocity, T);

  // Correct the target pose by a displacement predicted from the velocity.
    const auto	d    = dt.toSec();
    auto	pose = target_pose;
    pose.pose.position.x -= d * velocity.x;
    pose.pose.position.y -= d * velocity.y;
    pose.pose.position.z -= d * velocity.z;

    return  pose;
  //return nullptr;

  // // Convert velocity in twist to the target frame.
  //   const auto	twist = getTwist();
  //   vector3_t	velocity(twist.twist.linear.x,
  // 			 twist.twist.linear.y,
  // 			 twist.twist.linear.z);
  //   const auto	T = getFrameTransform(target_pose.header.frame_id,
  // 				      twist.header.frame_id);
  //   velocity = T * velocity;

  // // Correct the target pose by displacement predicted from the veloity.
  //   const auto	d = dt.toSec();
  //   vector3_t	dx = d * velocity;
  //   auto	pose = target_pose;
  //   // pose.pose.position.x -= dx(0);
  //   // pose.pose.position.y -= dx(1);
  //   // pose.pose.position.z -= dx(2);
  //   std::cerr << "*** dx = (" << dx.transpose() << ')' << std::endl;

  //   return pose;
#else
  // Get transform to base frame.
    const auto	twist = getTwist();
    const auto	T     = getFrameTransform(twist.header.frame_id,
					  target_pose.header.frame_id);

  // Get transform produced from twist.
    const vector3_t	angular(twist.twist.angular.x,
				twist.twist.angular.y,
				twist.twist.angular.z);
    const auto		d = dt.toSec();
    auto		dT = Servo::isometry3_t::Identity();
    // dT.linear()	     = angle_axis_t(d * angular.norm(), angular.normalized())
    // 		      .toRotationMatrix();
    dT.translation() = vector3_t(d * twist.twist.linear.x,
				 d * twist.twist.linear.y,
				 d * twist.twist.linear.z);
    // const auto	aa = angle_axis_t(d * angular.norm(), angular.normalized());
    // std::cerr << "*** aa.axis  = " << aa.axis().transpose() << std::endl;
    // std::cerr << "*** aa.angle = " << aa.angle() << std::endl;
    // std::cerr << "*** dT.trans = " << dT.translation().transpose()
    // 	      << std::endl;

  // Correct target pose by twist.
    auto	dS = tf2::eigenToTransform(T.inverse() * dT.inverse() * T);
    dS.header	      = target_pose.header;
    dS.child_frame_id = target_pose.header.frame_id;
    pose_t	pose;
    tf2::doTransform(target_pose, pose, dS);
    // const auto	dSS = T.inverse() * dT.inverse() * T;
    // std::cerr << "*** dS.trans = " << dSS.translation().transpose()
    // 	      << std::endl
    // 	      << "*** dS.liner = " << dSS.linear()
    // 	      << std::endl;

  //return  pose;
    return  nullptr;
#endif
}

OdometryFeedForwardServo::twist_t
OdometryFeedForwardServo::getTwist() const
{
    const std::lock_guard<std::mutex> lock(odometry_mtx_);

    twist_t	twist;
    twist.header.stamp	  = odometry_->header.stamp;
    twist.header.frame_id = odometry_->child_frame_id;
    twist.twist		  = odometry_->twist.twist;

    return twist;
}

void
OdometryFeedForwardServo::odometryCB(const odometry_cp& odometry)
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
    const std::string	logname("odometry_feedforward_servo");

    ros::init(argc, argv, logname);

    ros::NodeHandle				nh("~");
    aist_moveit_servo::OdometryFeedForwardServo	servo(nh, logname);
    servo.run();

    return 0;
}
