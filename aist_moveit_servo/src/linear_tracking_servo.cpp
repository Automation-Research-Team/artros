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
		LinearFeedForward(const Servo& servo)			;

    void	resetInput()						;
    bool	haveRecentInput(const ros::Duration& timeout)	const	;
    pose_t	ff_pose(const pose_t& target_pose,
			const ros::Duration& dt)		const	;

  private:
    void	velocityCB(const vector3_cp& velocity)			;

  private:
    const Servo&		servo_;
    ros::NodeHandle		nh_;
    const ros::Subscriber	velocity_sub_;
    vector3_t			velocity_;
    mutable std::mutex		velocity_mtx_;
};

LinearFeedForward::LinearFeedForward(const Servo& servo)
    :servo_(servo),
     nh_(servo_.nodeHandle()),
     velocity_sub_(nh_.subscribe("/velocity", 1,
				 &LinearFeedForward::velocityCB, this)),
     velocity_(),
     velocity_mtx_()
{
}

void
LinearFeedForward::resetInput()
{
    const std::lock_guard<std::mutex>	lock(velocity_mtx_);

    velocity_		   = vector3_t();
    velocity_.header.stamp = ros::Time(0);
}

bool
LinearFeedForward::haveRecentInput(const ros::Duration& timeout) const
{
    const std::lock_guard<std::mutex>	lock(velocity_mtx_);

    return (ros::Time::now() - velocity_.header.stamp < timeout);
}

LinearFeedForward::pose_t
LinearFeedForward::ff_pose(const pose_t& target_pose,
			   const ros::Duration& dt) const
{
    vector3_t	velocity;
    {
	const std::lock_guard<std::mutex>	lock(velocity_mtx_);

	velocity = velocity_;
    }

  // Convert the subscribed velocity to the frame describing the target pose.
    auto	T = tf2::eigenToTransform(servo_.getFrameTransform(
					      target_pose.header.frame_id,
					      velocity.header.frame_id));
    T.header	     = target_pose.header;
    T.child_frame_id = velocity.header.frame_id;
    tf2::doTransform(velocity, velocity, T);

  // Correct the target pose by a displacement predicted from the velocity.
    const auto	d    = dt.toSec();
    auto	pose = target_pose;
    pose.pose.position.x += d * velocity.vector.x;
    pose.pose.position.y += d * velocity.vector.y;
    pose.pose.position.z += d * velocity.vector.z;

    return  pose;
}

void
LinearFeedForward::velocityCB(const vector3_cp& velocity)
{
    const std::lock_guard<std::mutex> lock(velocity_mtx_);

    velocity_ = *velocity;
}

}	// namespace aist_moveit_servo

/************************************************************************
*  main function							*
************************************************************************/
int
main(int argc, char* argv[])
{
    using namespace aist_moveit_servo;
    
    const std::string	logname("linear_tracking_servo");

    ros::init(argc, argv, logname);

    ros::NodeHandle				nh("~");
    PoseTrackingServo<LinearFeedForward>	servo(nh, "robot_description",
						      logname);
    servo.run();

    return 0;
}
