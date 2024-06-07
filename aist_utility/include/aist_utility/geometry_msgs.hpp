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
#pragma once

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <tf2/convert.h>
#include <aist_utility/tf2.hpp>
#include <yaml-cpp/yaml.h>

namespace aist_utility
{
/*
 *  Arithmetic operators on Vector3
 */
inline geometry_msgs::msg::Vector3
operator +(const geometry_msgs::msg::Vector3& p,
	   const geometry_msgs::msg::Vector3& q)
{
    geometry_msgs::msg::Vector3	ret;
    ret.x = p.x	+ q.x;
    ret.y = p.y	+ q.y;
    ret.z = p.z	+ q.z;

    return ret;
}

inline geometry_msgs::msg::Vector3
operator -(const geometry_msgs::msg::Vector3& p,
	   const geometry_msgs::msg::Vector3& q)
{
    geometry_msgs::msg::Vector3	ret;
    ret.x = p.x - q.x;
    ret.y = p.y - q.y;
    ret.z = p.z - q.z;

    return ret;
}

inline geometry_msgs::msg::Vector3
operator *(double c, const geometry_msgs::msg::Vector3& p)
{
    geometry_msgs::msg::Vector3	ret;
    ret.x = c * p.x;
    ret.y = c * p.y;
    ret.z = c * p.z;

    return ret;
}

inline geometry_msgs::msg::Vector3
zero(geometry_msgs::msg::Vector3)
{
    geometry_msgs::msg::Vector3	ret;
    ret.x = 0;
    ret.y = 0;
    ret.z = 0;

    return ret;
}

/*
 *  Arithmetic operators on Point
 */
inline geometry_msgs::msg::Point
operator +(const geometry_msgs::msg::Point& p,
	   const geometry_msgs::msg::Point& q)
{
    geometry_msgs::msg::Point	ret;
    ret.x = p.x	+ q.x;
    ret.y = p.y	+ q.y;
    ret.z = p.z	+ q.z;

    return ret;
}

inline geometry_msgs::msg::Point
operator -(const geometry_msgs::msg::Point& p,
	   const geometry_msgs::msg::Point& q)
{
    geometry_msgs::msg::Point	ret;
    ret.x = p.x - q.x;
    ret.y = p.y - q.y;
    ret.z = p.z - q.z;

    return ret;
}

inline geometry_msgs::msg::Point
operator *(double c, const geometry_msgs::msg::Point& p)
{
    geometry_msgs::msg::Point	ret;
    ret.x = c * p.x;
    ret.y = c * p.y;
    ret.z = c * p.z;

    return ret;
}

inline geometry_msgs::msg::Point
operator *(const geometry_msgs::msg::Point& p, double c)
{
    return c * p;
}

inline geometry_msgs::msg::Point
zero(geometry_msgs::msg::Point)
{
    geometry_msgs::msg::Point	ret;
    ret.x = 0;
    ret.y = 0;
    ret.z = 0;

    return ret;
}

/*
 *  Arithmetic operators on Quaternion
 */
inline geometry_msgs::msg::Quaternion
operator +(const geometry_msgs::msg::Quaternion& p,
	   const geometry_msgs::msg::Quaternion& q)
{
    geometry_msgs::msg::Quaternion	ret;
    ret.x = p.x	+ q.x;
    ret.y = p.y	+ q.y;
    ret.z = p.z	+ q.z;
    ret.w = p.w	+ q.w;

    return ret;
}

inline geometry_msgs::msg::Quaternion
operator -(const geometry_msgs::msg::Quaternion& p,
	   const geometry_msgs::msg::Quaternion& q)
{
    geometry_msgs::msg::Quaternion	ret;
    ret.x = p.x	- q.x;
    ret.y = p.y	- q.y;
    ret.z = p.z	- q.z;
    ret.w = p.w	- q.w;

    return ret;
}

inline geometry_msgs::msg::Quaternion
operator *(double c, const geometry_msgs::msg::Quaternion& p)
{
    geometry_msgs::msg::Quaternion	ret;
    ret.x = c * p.x;
    ret.y = c * p.y;
    ret.z = c * p.z;
    ret.w = c * p.w;

    return ret;
}

inline geometry_msgs::msg::Quaternion
operator *(const geometry_msgs::msg::Quaternion& p, double c)
{
    return c * p;
}

inline geometry_msgs::msg::Quaternion
zero(geometry_msgs::msg::Quaternion)
{
    geometry_msgs::msg::Quaternion	ret;
    ret.x = 0;
    ret.y = 0;
    ret.z = 0;
    ret.w = 0;

    return ret;
}

inline geometry_msgs::msg::Quaternion
identity(geometry_msgs::msg::Quaternion)
{
    geometry_msgs::msg::Quaternion	ret;
    ret.x = 0;
    ret.y = 0;
    ret.z = 0;
    ret.w = 1;

    return ret;
}

inline void
normalize(geometry_msgs::msg::Quaternion& q)
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
inline geometry_msgs::msg::Pose
operator +(const geometry_msgs::msg::Pose& p,
	   const geometry_msgs::msg::Pose& q)
{
    geometry_msgs::msg::Pose	ret;
    ret.position    = p.position    + q.position;
    ret.orientation = p.orientation + q.orientation;

    return ret;
}

inline geometry_msgs::msg::Pose
operator -(const geometry_msgs::msg::Pose& p,
	   const geometry_msgs::msg::Pose& q)
{
    geometry_msgs::msg::Pose	ret;
    ret.position    = p.position    - q.position;
    ret.orientation = p.orientation - q.orientation;

    return ret;
}

inline geometry_msgs::msg::Pose
operator *(double c, const geometry_msgs::msg::Pose& p)
{
    geometry_msgs::msg::Pose	ret;
    ret.position    = c * p.position;
    ret.orientation = c * p.orientation;

    return ret;
}

inline geometry_msgs::msg::Pose
operator *(const geometry_msgs::msg::Pose& p, double c)
{
    return c * p;
}

inline geometry_msgs::msg::Pose
zero(geometry_msgs::msg::Pose)
{
    geometry_msgs::msg::Pose	ret;
    ret.position    = zero(geometry_msgs::msg::Point());
    ret.orientation = zero(geometry_msgs::msg::Quaternion());

    return ret;
}

inline geometry_msgs::msg::Pose
identity(geometry_msgs::msg::Pose)
{
    geometry_msgs::msg::Pose	ret;
    ret.position    = zero(geometry_msgs::msg::Point());
    ret.orientation = identity(geometry_msgs::msg::Quaternion());

    return ret;
}

/*
 *  Arithmetic operators on Transform
 */
inline geometry_msgs::msg::Transform
identity(geometry_msgs::msg::Transform)
{
    geometry_msgs::msg::Transform	ret;
    ret.translation = zero(geometry_msgs::msg::Vector3());
    ret.rotation    = identity(geometry_msgs::msg::Quaternion());

    return ret;
}

/*
 *  I/O operators
 */
inline std::ostream&
operator <<(std::ostream& out, const geometry_msgs::msg::Vector3& v)
{
    return out << '[' << v.x << ',' << v.y << ',' << v.z << ']';
}

inline std::ostream&
operator <<(std::ostream& out, const geometry_msgs::msg::Point& p)
{
    return out << '[' << p.x << ',' << p.y << ',' << p.z << ']';
}

inline std::ostream&
operator <<(std::ostream& out, const geometry_msgs::msg::Quaternion& q)
{
    tf2::Quaternion	qq;
    tf2::fromMsg(q, qq);

    return out << qq;
}

inline std::ostream&
operator <<(std::ostream& out, const geometry_msgs::msg::Pose& pose)
{
    return out << '[' << pose.position << "; " << pose.orientation << ']';
}

inline std::ostream&
operator <<(std::ostream& out, const geometry_msgs::msg::Transform& transform)
{
    return out << '['  << transform.translation
	       << "; " << transform.rotation << ']';
}

inline std::ostream&
operator <<(std::ostream& out, const geometry_msgs::msg::Twist& twist)
{
    return out << '['  << twist.linear << "; " << twist.angular << ']';
}

/*
 *  Conversion between Transform and Pose
 */
inline geometry_msgs::msg::Pose
toPose(const geometry_msgs::msg::Transform& transform)
{
    geometry_msgs::msg::Pose	pose;
    pose.position.x  = transform.translation.x;
    pose.position.y  = transform.translation.y;
    pose.position.z  = transform.translation.z;
    pose.orientation = transform.rotation;

    return pose;
}

inline geometry_msgs::msg::PoseStamped
toPose(const geometry_msgs::msg::TransformStamped& transform)
{
    geometry_msgs::msg::PoseStamped	pose;
    pose.header = transform.header;
    pose.pose	= toPose(transform.transform);

    return pose;
}

inline geometry_msgs::msg::Transform
toTransform(const geometry_msgs::msg::Pose& pose)
{
    geometry_msgs::msg::Transform	transform;
    transform.translation.x = pose.position.x;
    transform.translation.y = pose.position.y;
    transform.translation.z = pose.position.z;
    transform.rotation	    = pose.orientation;

    return transform;
}

inline geometry_msgs::msg::TransformStamped
toTransform(const geometry_msgs::msg::PoseStamped& pose,
	    const std::string& child_frame_id)
{
    geometry_msgs::msg::TransformStamped	transform;
    transform.header	     = pose.header;
    transform.child_frame_id = child_frame_id;
    transform.transform	     = toTransform(pose.pose);

    return transform;
}

/*
 *  Emit geometry messages in YAML format
 */
inline YAML::Emitter&
operator <<(YAML::Emitter& emitter, const builtin_interfaces::msg::Time& time)
{
    return emitter << YAML::BeginMap
		   << YAML::Key << "secs"  << YAML::Value << time.sec
		   << YAML::Key << "nsecs" << YAML::Value << time.nanosec
		   << YAML::EndMap;
}

inline YAML::Emitter&
operator <<(YAML::Emitter& emitter, const std_msgs::msg::Header& header)
{
    return emitter << YAML::BeginMap
		   << YAML::Key << "stamp"    << YAML::Value << header.stamp
		   << YAML::Key << "frame_id" << YAML::Value << header.frame_id
		   << YAML::EndMap;
}

inline YAML::Emitter&
operator <<(YAML::Emitter& emitter, const geometry_msgs::msg::Vector3& v)
{
    return emitter << YAML::BeginMap
		   << YAML::Key << "x" << YAML::Value << v.x
		   << YAML::Key << "y" << YAML::Value << v.y
		   << YAML::Key << "z" << YAML::Value << v.z
		   << YAML::EndMap;
}

inline YAML::Emitter&
operator <<(YAML::Emitter& emitter, const geometry_msgs::msg::Quaternion& q)
{
    return emitter << YAML::BeginMap
		   << YAML::Key << "x" << YAML::Value << q.x
		   << YAML::Key << "y" << YAML::Value << q.y
		   << YAML::Key << "z" << YAML::Value << q.z
		   << YAML::Key << "w" << YAML::Value << q.w
		   << YAML::EndMap;
}

inline YAML::Emitter&
operator <<(YAML::Emitter& emitter, const geometry_msgs::msg::Point& p)
{
    return emitter << YAML::BeginMap
		   << YAML::Key << "x" << YAML::Value << p.x
		   << YAML::Key << "y" << YAML::Value << p.y
		   << YAML::Key << "z" << YAML::Value << p.z
		   << YAML::EndMap;
}

inline YAML::Emitter&
operator <<(YAML::Emitter& emitter, const geometry_msgs::msg::Pose& pose)
{
    return emitter << YAML::BeginMap
		   << YAML::Key   << "position"
		   << YAML::Value << pose.position
		   << YAML::Key   << "orientation"
		   << YAML::Value << pose.orientation
		   << YAML::EndMap;
}

inline YAML::Emitter&
operator <<(YAML::Emitter& emitter, const geometry_msgs::msg::Twist& twist)
{
    return emitter << YAML::BeginMap
		   << YAML::Key << "linear"  << YAML::Value << twist.linear
		   << YAML::Key << "angular" << YAML::Value << twist.angular
		   << YAML::EndMap;
}

inline YAML::Emitter&
operator <<(YAML::Emitter& emitter, const geometry_msgs::msg::Transform& t)
{
    return emitter << YAML::BeginMap
		   << YAML::Key   << "translation"
		   << YAML::Value << t.translation
		   << YAML::Key   << "rotation"
		   << YAML::Value << t.rotation
		   << YAML::EndMap;
}

inline YAML::Emitter&
operator <<(YAML::Emitter& emitter, const geometry_msgs::msg::TransformStamped& t)
{
    return emitter << YAML::BeginMap
		   << YAML::Key   << "header"
		   << YAML::Value << t.header
		   << YAML::Key   << "child_frame_id"
		   << YAML::Value << t.child_frame_id
		   << YAML::Key   << "transform"
		   << YAML::Value << t.transform
		   << YAML::EndMap;
}

template <class MSG> YAML::Emitter&
operator <<(YAML::Emitter& emitter, const std::vector<MSG>& msgs)
{
    emitter << YAML::BeginSeq;
    for (const auto& msg : msgs)
	aist_utility::operator <<(emitter, msg);
    return emitter << YAML::EndSeq;
}

namespace detail
{
template <class MSG> inline YAML::Emitter&
put(YAML::Emitter& emitter,
    const std_msgs::msg::Header& header, const MSG& msg, const std::string& key)
{
    using aist_utility::operator <<;

    emitter << YAML::BeginMap
	    << YAML::Key << "header" << YAML::Value << header
	    << YAML::Key << key      << YAML::Value;
    return aist_utility::operator <<(emitter, msg) << YAML::EndMap;
}
}

inline YAML::Emitter&
operator <<(YAML::Emitter& emitter, const geometry_msgs::msg::Vector3Stamped& v)
{
    return detail::put(emitter, v.header, v.vector, "vector");
}

inline YAML::Emitter&
operator <<(YAML::Emitter& emitter, const geometry_msgs::msg::QuaternionStamped& q)
{
    return detail::put(emitter, q.header, q.quaternion, "quaternion");
}

inline YAML::Emitter&
operator <<(YAML::Emitter& emitter, const geometry_msgs::msg::PointStamped& p)
{
    return detail::put(emitter, p.header, p.point, "point");
}

inline YAML::Emitter&
operator <<(YAML::Emitter& emitter, const geometry_msgs::msg::PoseStamped& pose)
{
    return detail::put(emitter, pose.header, pose.pose, "pose");
}

inline YAML::Emitter&
operator <<(YAML::Emitter& emitter, const geometry_msgs::msg::PoseArray& poses)
{
    return detail::put(emitter, poses.header, poses.poses, "poses");
}

inline YAML::Emitter&
operator <<(YAML::Emitter& emitter, const geometry_msgs::msg::TwistStamped& twist)
{
    return detail::put(emitter, twist.header, twist.twist, "twist");
}

/*
 *  Restore YAML as geometry messages
 */
inline void
operator >>(const YAML::Node& node, builtin_interfaces::msg::Time& time)
{
    time.sec     = node["secs"] .as< int32_t>();
    time.nanosec = node["nsecs"].as<uint32_t>();
}

inline void
operator >>(const YAML::Node& node, std_msgs::msg::Header& header)
{
    node["stamp"] >> header.stamp;
    header.frame_id = node["frame_id"].as<std::string>();
}

inline void
operator >>(const YAML::Node& node, geometry_msgs::msg::Vector3& vector)
{
    vector.x = node["x"].as<double>();
    vector.y = node["y"].as<double>();
    vector.z = node["z"].as<double>();
}

inline void
operator >>(const YAML::Node& node, geometry_msgs::msg::Quaternion& q)
{
    q.x = node["x"].as<double>();
    q.y = node["y"].as<double>();
    q.z = node["z"].as<double>();
    q.w = node["w"].as<double>();
}

inline void
operator >>(const YAML::Node& node, geometry_msgs::msg::Transform& t)
{
    node["translation"] >> t.translation;
    node["rotation"]    >> t.rotation;
}

inline void
operator >>(const YAML::Node& node, geometry_msgs::msg::Point& point)
{
    point.x = node["x"].as<double>();
    point.y = node["y"].as<double>();
    point.z = node["z"].as<double>();
}

inline void
operator >>(const YAML::Node& node, geometry_msgs::msg::Pose& pose)
{
    node["position"]    >> pose.position;
    node["orientation"] >> pose.orientation;
}

inline void
operator >>(const YAML::Node& node, geometry_msgs::msg::Twist& twist)
{
    node["linear"]  >> twist.linear;
    node["angular"] >> twist.angular;
}

inline void
operator >>(const YAML::Node& node, geometry_msgs::msg::Vector3Stamped& v)
{
    node["header"] >> v.header;
    node["vector"] >> v.vector;
}

inline void
operator >>(const YAML::Node& node, geometry_msgs::msg::QuaternionStamped& q)
{
    node["header"]     >> q.header;
    node["quaternion"] >> q.quaternion;
}

inline void
operator >>(const YAML::Node& node, geometry_msgs::msg::PointStamped& p)
{
    node["header"] >> p.header;
    node["point"]  >> p.point;
}

inline void
operator >>(const YAML::Node& node, geometry_msgs::msg::PoseStamped& pose)
{
    node["header"] >> pose.header;
    node["pose"]   >> pose.pose;
}

inline void
operator >>(const YAML::Node& node, geometry_msgs::msg::TwistStamped& twist)
{
    node["header"] >> twist.header;
    node["twist"]  >> twist.twist;
}

inline void
operator >>(const YAML::Node& node, geometry_msgs::msg::TransformStamped& t)
{
    node["header"] >> t.header;
    t.child_frame_id = node["child_frame_id"].as<std::string>();
    node["transform"]  >> t.transform;
}

template <class MSG> void
operator >>(const YAML::Node& node, std::vector<MSG>& msgs)
{
    if (node.IsSequence())
	for (const auto& sub_node : node)
	{
	    MSG	msg;
	    aist_utility::operator >>(sub_node, msg);
	    msgs.push_back(msg);
	}
}

inline void
operator >>(const YAML::Node& node, geometry_msgs::msg::PoseArray& poses)
{
    node["header"] >> poses.header;
    aist_utility::operator >>(node["poses"], poses.poses);
}

}	// namespace aist_utility

