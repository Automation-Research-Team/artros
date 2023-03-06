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
 *  \file	pose_tracking_servo.cpp
 *  \brief	ROS pose tracker of aist_moveit_servo::PoseTracking type
 */
#include <ros/ros.h>
#include <geometry_msgs::Pose.h>

// Conventions:
// Calculations are done in the planning_frame_ unless otherwise noted.

namespace
{
constexpr char		LOGNAME[]	  = "pose_tracking_servo";
constexpr double	ROS_STARTUP_WAIT  = 10;		// sec
const	  ros::Duration	DEFAULT_INPUT_TIMEOUT{0.5};	// sec
}	// anonymous namespace

namespace geometry_msgs
{
Pose
operator +(const Pose& a, const Pose& b)
{
    Pose	ret;
    ret.position.x    = a.position.x	+ b.position.x;
    ret.position.y    = a.position.y	+ b.position.y;
    ret.position.z    = a.position.z	+ b.position.z;
    ret.orientation.x = a.orientation.x	+ b.orientation.x;
    ret.orientation.y = a.orientation.y	+ b.orientation.y;
    ret.orientation.z = a.orientation.z	+ b.orientation.z;
    ret.orientation.w = a.orientation.w	+ b.orientation.w;

    return ret;
}

Pose
operator -(const Pose& a, const Pose& b)
{
    Pose	ret;
    ret.position.x    = a.position.x	- b.position.x;
    ret.position.y    = a.position.y	- b.position.y;
    ret.position.z    = a.position.z	- b.position.z;
    ret.orientation.x = a.orientation.x	- b.orientation.x;
    ret.orientation.y = a.orientation.y	- b.orientation.y;
    ret.orientation.z = a.orientation.z	- b.orientation.z;
    ret.orientation.w = a.orientation.w	- b.orientation.w;

    return ret;
}

Pose
operator *(double c, const Pose& a)
{
    Pose	ret;
    ret.position.x    = c * a.position.x;
    ret.position.y    = c * a.position.y;
    ret.position.z    = c * a.position.z;
    ret.orientation.x = c * a.orientation.x;
    ret.orientation.y = c * a.orientation.y;
    ret.orientation.z = c * a.orientation.z;
    ret.orientation.w = c * a.orientation.w;

    return ret;
}

Pose
zero(Pose)
{
    Pose	ret;
    ret.position.x    = 0;
    ret.position.y    = 0;
    ret.position.z    = 0;
    ret.orientation.x = 0;
    ret.orientation.y = 0;
    ret.orientation.z = 0;
    ret.orientation.w = 0;

    return ret;
}

Point
operator +(const Point& a, const Point& b)
{
    Point	ret;
    ret.x = a.x	+ b.x;
    ret.y = a.y	+ b.y;
    ret.z = a.z	+ b.z;

    return ret;
}

Point
operator -(const Point& a, const Point& b)
{
    Point	ret;
    ret.x = a.x - b.x;
    ret.y = a.y - b.y;
    ret.z = a.z - b.z;

    return ret;
}

Point
operator *(double c, const Point& a)
{
    Point	ret;
    ret.x = c * a.x;
    ret.y = c * a.y;
    ret.z = c * a.z;

    return ret;
}

Point
zero(Point)
{
    Point	ret;
    ret.x = 0;
    ret.y = 0;
    ret.z = 0;

    return ret;
}

}	// namespace geometry_msgs

namespace
{
/************************************************************************
*  static functions							*
************************************************************************/
std::ostream&
operator <<(std::ostream& out, const geometry_msgs::Pose& pose)
{
    return out << pose.position.x << ' '
	       << pose.position.y << ' '
	       << pose.position.z << ';'
	       << pose.orientation.x << ' '
	       << pose.orientation.y << ' '
	       << pose.orientation.z << ' '
	       << pose.orientation.w;
}

std::ostream&
operator <<(std::ostream& out, const Eigen::Isometry3d& transform)
{
    const Eigen::Quaterniond	q(transform.rotation());

    return out << transform.translation()(0) << ' '
	       << transform.translation()(1) << ' '
	       << transform.translation()(2) << ';'
	       << q.x() << ' '
	       << q.y() << ' '
	       << q.z() << ' '
	       << q.w();
}

std::ostream&
operator <<(std::ostream& out, const geometry_msgs::Twist& twist)
{
    return out << twist.linear.x << ' '
	       << twist.linear.y << ' '
	       << twist.linear.z << ';'
	       << twist.angular.x << ' '
	       << twist.angular.y << ' '
	       << twist.angular.z;
}

void
normalize(geometry_msgs::Quaternion& q)
{
    const auto	norm1 = 1/std::sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w);
    q.x *= norm1;
    q.y *= norm1;
    q.z *= norm1;
    q.w *= norm1;
}

}	// anonymous namespace

/************************************************************************
*  main function							*
************************************************************************/
int
main(int argc, char* argv[])
{
    ros::init(argc, argv, LOGNAME);

    ros::NodeHandle	nh("~");
    aist_moveit_servo::PoseTrackingServo	servo(nh);
    servo.run();

    return 0;
}
