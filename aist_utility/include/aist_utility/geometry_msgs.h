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
inline std::ostream&
operator <<(std::ostream& out, const geometry_msgs::Point& p)
{
    return out << '[' << p.x << ',' << p.y << ',' << p.z << ']';
}

inline std::ostream&
operator <<(std::ostream& out, const geometry_msgs::Vector3& v)
{
    return out << '[' << v.x << ',' << v.y << ',' << v.z << ']';
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

}	// namespace aist_utility
#endif	// !AIST_UTILITY_GEOMETRY_MSGS_H
