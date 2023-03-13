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
 *  \file	linear_tracking_servo.cpp
 *  \brief	ROS pose tracker of aist_moveit_servo::PoseTracking type
 *  \author	Toshio UESHIBA
 */
#include <mutex>
#include <thread>
#include <aist_moveit_servo/pose_tracking_servo.h>

namespace aist_moveit_servo
{
class LinearFeedForward
{
  private:
    using vector3_t	 = geometry_msgs::Vector3Stamped;
    using vector3_cp	 = geometry_msgs::Vector3StampedConstPtr;
    using pose_t	 = geometry_msgs::PoseStamped;

  public:
		LinearFeedForward(const ros::NodeHandle& nh)		;

    void	resetInput()						;
    bool	haveRecentInput(const ros::Duration& timeout)	const	;
    pose_t	ff_pose(const pose_t& desired_pose,
			const ros::Duration& dt)		const	;

  private:
    void	velocityCB(const vector3_cp& velocity)			;

  private:
    ros::NodeHandle		nh_;
    const ros::Subscriber	velocity_sub_;
    veclocity_t			velocity_;
    mutable std::mutex		velocity_mtx_;
};

LinearFeedForward::LinearFeedForward(const ros::NodeHandle& nh)
    :nh_(nh),
     velocity_sub_(nh_.subscribe("/velocity",
				 &LinearFeedForward::velocityCB, this)),
     velocity_(),
     velocity_mtx_()
{
}

void
LinearFeedForward::resetTargetPose()
{
    const std::lock_guard<std::mutex>	lock(velocity_mtx_);

    velocity_		   = veloccity_t();
    velocity_.header.stamp = ros::Time(0);
}

bool
LinearFeedForward::haveRecentInput(const ros::Duration& timeout) const
{
    const std::lock_guard<std::mutex>	lock(velocity_mtx_);

    return (ros::Time::now() - velocity_.header.stamp < timeout);
}

void
LinearFeedForward::velocityCB(const vector3_cp& velocity)
{
    const std::lock_guard<std::mutex> lock(input_mutex_);

    velocity_ = *velocity;
}

}	// namespace aist_moveit_servo

/************************************************************************
*  main function							*
************************************************************************/
int
main(int argc, char* argv[])
{
    const std::string	logname("conveyor_tracking_servo");

    ros::init(argc, argv, logname);

    ros::NodeHandle	nh("~");
    aist_moveit_servo::PoseTrackingServo<>	servo(nh, logname);
    servo.run();

    return 0;
}
