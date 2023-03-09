// Software License Agreement (BSD License)
//
// Copyright (c) 2021, National Institute of Advanced Industrial Science and Technology (AIST)
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//  * Neither the name of National Institute of Advanced Industrial
//    Science and Technology (AIST) nor the names of its contributors
//    may be used to endorse or promote products derived from this software
//    without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// Author: Toshio Ueshiba
//
/*!
 *  \file	geometry_msgs.h
 *  \author	Toshio Ueshiba
 *  \brief	Utilities
 */
#ifndef AIST_UTILITY_GEOMETRY_MSGS_H
#define AIST_UTILITY_GEOMETRY_MSGS_H

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Transform.h>
#include <aist_utility/tf.h>

namespace aist_utility
{
/*
 *  Arithmetic operators on Vector3
 */
inline geometry_msgs::Vector3
operator +(const geometry_msgs::Vector3& p, const geometry_msgs::Vector3& q)
{
    geometry_msgs::Vector3	ret;
    ret.x = p.x	+ q.x;
    ret.y = p.y	+ q.y;
    ret.z = p.z	+ q.z;

    return ret;
}

inline geometry_msgs::Vector3
operator -(const geometry_msgs::Vector3& p, const geometry_msgs::Vector3& q)
{
    geometry_msgs::Vector3	ret;
    ret.x = p.x - q.x;
    ret.y = p.y - q.y;
    ret.z = p.z - q.z;

    return ret;
}

inline geometry_msgs::Vector3
operator *(double c, const geometry_msgs::Vector3& p)
{
    geometry_msgs::Vector3	ret;
    ret.x = c * p.x;
    ret.y = c * p.y;
    ret.z = c * p.z;

    return ret;
}

/*
 *  Arithmetic operators on Point
 */
inline geometry_msgs::Point
operator +(const geometry_msgs::Point& p, const geometry_msgs::Point& q)
{
    geometry_msgs::Point	ret;
    ret.x = p.x	+ q.x;
    ret.y = p.y	+ q.y;
    ret.z = p.z	+ q.z;

    return ret;
}

inline geometry_msgs::Point
operator -(const geometry_msgs::Point& p, const geometry_msgs::Point& q)
{
    geometry_msgs::Point	ret;
    ret.x = p.x - q.x;
    ret.y = p.y - q.y;
    ret.z = p.z - q.z;

    return ret;
}

inline geometry_msgs::Point
operator *(double c, const geometry_msgs::Point& p)
{
    geometry_msgs::Point	ret;
    ret.x = c * p.x;
    ret.y = c * p.y;
    ret.z = c * p.z;

    return ret;
}

inline geometry_msgs::Point
operator *(const geometry_msgs::Point& p, double c)
{
    return c * p;
}

inline geometry_msgs::Point
zero(geometry_msgs::Point)
{
    geometry_msgs::Point	ret;
    ret.x = 0;
    ret.y = 0;
    ret.z = 0;

    return ret;
}

/*
 *  Arithmetic operators on Quaternion
 */
inline geometry_msgs::Quaternion
operator +(const geometry_msgs::Quaternion& p,
	   const geometry_msgs::Quaternion& q)
{
    geometry_msgs::Quaternion	ret;
    ret.x = p.x	+ q.x;
    ret.y = p.y	+ q.y;
    ret.z = p.z	+ q.z;
    ret.w = p.w	+ q.w;

    return ret;
}

inline geometry_msgs::Quaternion
operator -(const geometry_msgs::Quaternion& p,
	   const geometry_msgs::Quaternion& q)
{
    geometry_msgs::Quaternion	ret;
    ret.x = p.x	- q.x;
    ret.y = p.y	- q.y;
    ret.z = p.z	- q.z;
    ret.w = p.w	- q.w;

    return ret;
}

inline geometry_msgs::Quaternion
operator *(double c, const geometry_msgs::Quaternion& p)
{
    geometry_msgs::Quaternion	ret;
    ret.x = c * p.x;
    ret.y = c * p.y;
    ret.z = c * p.z;
    ret.w = c * p.w;

    return ret;
}

inline geometry_msgs::Quaternion
operator *(const geometry_msgs::Quaternion& p, double c)
{
    return c * p;
}

inline geometry_msgs::Quaternion
zero(geometry_msgs::Quaternion)
{
    geometry_msgs::Quaternion	ret;
    ret.x = 0;
    ret.y = 0;
    ret.z = 0;
    ret.w = 0;

    return ret;
}

inline void
normalize(geometry_msgs::Quaternion& q)
{
    const auto	norm1 = 1/std::sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w);
    q.x *= norm1;
    q.y *= norm1;
    q.z *= norm1;
    q.w *= norm1;
}

/*
 *  Arithmetic operators on Pose
 */
inline geometry_msgs::Pose
operator +(const geometry_msgs::Pose& p, const geometry_msgs::Pose& q)
{
    geometry_msgs::Pose	ret;
    ret.position    = p.position    + q.position;
    ret.orientation = p.orientation + q.orientation;

    return ret;
}

inline geometry_msgs::Pose
operator -(const geometry_msgs::Pose& p, const geometry_msgs::Pose& q)
{
    geometry_msgs::Pose	ret;
    ret.position    = p.position    - q.position;
    ret.orientation = p.orientation - q.orientation;

    return ret;
}

inline geometry_msgs::Pose
operator *(double c, const geometry_msgs::Pose& p)
{
    geometry_msgs::Pose	ret;
    ret.position    = c * p.position;
    ret.orientation = c * p.orientation;

    return ret;
}

inline geometry_msgs::Pose
operator *(const geometry_msgs::Pose& p, double c)
{
    return c * p;
}

inline geometry_msgs::Pose
zero(geometry_msgs::Pose)
{
    geometry_msgs::Pose	ret;
    ret.position    = zero(geometry_msgs::Point());
    ret.orientation = zero(geometry_msgs::Quaternion());

    return ret;
}

/*
 *  I/O operators
 */
inline std::ostream&
operator <<(std::ostream& out, const geometry_msgs::Vector3& v)
{
    return out << '[' << v.x << ',' << v.y << ',' << v.z << ']';
}

inline std::ostream&
operator <<(std::ostream& out, const geometry_msgs::Point& p)
{
    return out << '[' << p.x << ',' << p.y << ',' << p.z << ']';
}

inline std::ostream&
operator <<(std::ostream& out, const geometry_msgs::Quaternion& q)
{
    tf::Quaternion	qq;
    tf::quaternionMsgToTF(q, qq);

    return out << qq;
}

inline std::ostream&
operator <<(std::ostream& out, const geometry_msgs::Pose& pose)
{
    return out << '[' << pose.position << "; " << pose.orientation << ']';
}

inline std::ostream&
operator <<(std::ostream& out, const geometry_msgs::Transform& transform)
{
    return out << '['  << transform.translation
	       << "; " << transform.rotation << ']';
}

/*
 *  Conversion betwenn Transform and Pose
 */
inline geometry_msgs::Pose
toPose(const geometry_msgs::Transform& transform)
{
    geometry_msgs::Pose	pose;
    pose.position.x  = transform.translation.x;
    pose.position.y  = transform.translation.y;
    pose.position.z  = transform.translation.z;
    pose.orientation = transform.rotation;

    return pose;
}

inline geometry_msgs::PoseStamped
toPose(const geometry_msgs::TransformStamped& transform)
{
    geometry_msgs::PoseStamped	pose;
    pose.header = transform.header;
    pose.pose	= toPose(transform.transform);

    return pose;
}

inline geometry_msgs::Transform
toTransform(const geometry_msgs::Pose& pose)
{
    geometry_msgs::Transform	transform;
    transform.translation.x = pose.position.x;
    transform.translation.y = pose.position.y;
    transform.translation.z = pose.position.z;
    transform.rotation	    = pose.orientation;

    return transform;
}

inline geometry_msgs::TransformStamped
toTransform(const geometry_msgs::PoseStamped& pose,
	    const std::string& child_frame_id)
{
    geometry_msgs::TransformStamped	transform;
    transform.header	     = pose.header;
    transform.child_frame_id = child_frame_id;
    transform.transform	     = toTransform(pose.pose);

    return transform;
}

}	// namespace aist_utility
#endif	// !AIST_UTILITY_GEOMETRY_MSGS_H
