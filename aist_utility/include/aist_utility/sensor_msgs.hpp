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
 *  \file	sensor_msgs.h
 *  \author	Toshio Ueshiba
 *  \brief	Utilities
 */
#pragma once

#include <array>
#include <algorithm>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <aist_utility/geometry_msgs.hpp>

namespace aist_utility
{
//! Get intensity value
template <class T>
inline float	intensity(const T& grey)	{ return grey; }
template <>
inline float	intensity(const std::array<uint8_t, 3>& rgb)
		{
		    return 0.299f*rgb[0] + 0.587f*rgb[1] + 0.114f*rgb[2];
		}

//! Set color value
template <class T> inline void
set_color(const sensor_msgs::PointCloud2Iterator<uint8_t>& p, const T& rgb)
{
    p[0] = rgb.b;
    p[1] = rgb.g;
    p[2] = rgb.r;
}
template <> inline void
set_color(const sensor_msgs::PointCloud2Iterator<uint8_t>& p,
	  const uint8_t& grey)
{
    p[0] = p[1] = p[2] = grey;
}

//! Get depth value in meters.
template <class T>
inline float	meters(T depth)			{ return depth; }
template <>
inline float	meters(uint16_t depth)		{ return 0.001f * depth; }

//! Get depth value in milimeters.
template <class T>
inline float	milimeters(T depth)		{ return 1000.0f * depth; }
template <>
inline float	milimeters(uint16_t depth)	{ return depth; }

//! Get pointer to the leftmost pixel of v-th scanline of the image.
template <class T> inline T*
ptr(sensor_msgs::msg::Image& image, int v)
{
    return reinterpret_cast<T*>(image.data.data() + v*image.step);
}

//! Get const pointer to the leftmost pixel of v-th scanline of the image.
template <class T> inline const T*
ptr(const sensor_msgs::msg::Image& image, int v)
{
    return reinterpret_cast<const T*>(image.data.data() + v*image.step);
}

inline sensor_msgs::msg::Image
create_empty_image(const rclcpp::Time& stamp, const std::string& frame,
		   const std::string& encoding
					=sensor_msgs::image_encodings::RGB8)
{
    using namespace	sensor_msgs::msg;

    Image	image;
    image.header.stamp	  = stamp;
    image.header.frame_id = frame;
    image.height	  = 1;
    image.width		  = 0;
    image.encoding	  = encoding;
    image.is_bigendian	  = false;
    image.step		  = 0;
    image.data.clear();

    return image;
}

//! Convert depth image to sequence of 3D points.
template <class T, class ITER, class UNIT> ITER
depth_to_points(const sensor_msgs::msg::CameraInfo& camera_info,
		const sensor_msgs::msg::Image& depth, ITER out, UNIT unit)
{
    cv::Mat_<float>	K(3, 3);
    std::copy_n(std::begin(camera_info.k), 9, K.begin());
    cv::Mat_<float>	D(1, 4);
    std::copy_n(std::begin(camera_info.d), 4, D.begin());

    cv::Mat_<cv::Point2f>	uv(depth.width, 1), xy(depth.width, 1);
    for (uint32_t u = 0; u < depth.width; ++u)
	uv(u).x = u;

    for (uint32_t v = 0; v < depth.height; ++v)
    {
	for (uint32_t u = 0; u < depth.width; ++u)
	    uv(u).y = v;

	cv::undistortPoints(uv, xy, K, D);

	auto	p = ptr<T>(depth, v);
	for (uint32_t u = 0; u < depth.width; ++u)
	{
	    const auto	d = unit(*p++);		// meters or milimeters
	    *out++ = {xy(u).x * d, xy(u).y * d, d};
	}
    }

    return out;
}

//! Extract 3D points from pointcloud.
template <class T=float, class ITER, class UNIT> ITER
pointcloud_to_points(const sensor_msgs::msg::PointCloud2& cloud,
		     ITER out, UNIT unit)
{
    sensor_msgs::PointCloud2ConstIterator<T>	xyz(cloud, "x");

    for (uint32_t v = 0; v < cloud.height; ++v)
	for (uint32_t u = 0; u < cloud.width; ++u)
	{
	    *out++ = {unit(xyz[0]), unit(xyz[1]), unit(xyz[2])};
	    ++xyz;
	}

    return out;
}

//! Extract RGB from pointcloud.
template <class ITER> ITER
pointcloud_to_rgb(const sensor_msgs::msg::PointCloud2& cloud, ITER out)
{
    sensor_msgs::PointCloud2ConstIterator<uint8_t>	bgr(cloud, "rgb");

    for (uint32_t v = 0; v < cloud.height; ++v)
	for (uint32_t u = 0; u < cloud.width; ++u)
	{
	    *out++ = {bgr[2], bgr[1], bgr[0]};
	    ++bgr;
	}

    return out;
}

template <class IN> sensor_msgs::msg::PointCloud2
create_pointcloud(IN in, IN ie,
		  const rclcpp::Time& stamp, const std::string& frame)
{
    using namespace	sensor_msgs;

    msg::PointCloud2	cloud;
    cloud.is_bigendian	  = false;
    cloud.is_dense	  = true;
    cloud.header.stamp	  = stamp;
    cloud.header.frame_id = frame;
    cloud.height	  = 1;
    cloud.width		  = std::distance(in, ie);

    PointCloud2Modifier	modifier(cloud);
    modifier.setPointCloud2Fields(3,
				  "x", 1, msg::PointField::FLOAT32,
				  "y", 1, msg::PointField::FLOAT32,
				  "z", 1, msg::PointField::FLOAT32);
    modifier.resize(cloud.width);
    cloud.row_step = cloud.width * cloud.point_step;

    PointCloud2Iterator<float>	out(cloud, "x");
    for (; in != ie; ++in, ++out)
    {
	using	std::get;

	const auto&	xyz = *in;
	out[0] = get<0>(xyz);
	out[1] = get<1>(xyz);
	out[2] = get<2>(xyz);
    }

    return cloud;
}

template <class T> sensor_msgs::msg::PointCloud2
create_pointcloud(const sensor_msgs::msg::CameraInfo& camera_info,
		  const sensor_msgs::msg::Image& depth, bool with_rgb=false)
{
    using namespace	sensor_msgs;

    msg::PointCloud2	cloud;
    cloud.header	= depth.header;
    cloud.height	= depth.height;
    cloud.width		= depth.width;
    cloud.is_bigendian	= false;
    cloud.is_dense	= false;

    PointCloud2Modifier	modifier(cloud);
    if (with_rgb)
	modifier.setPointCloud2Fields(4,
				      "x",   1, msg::PointField::FLOAT32,
				      "y",   1, msg::PointField::FLOAT32,
				      "z",   1, msg::PointField::FLOAT32,
				      "rgb", 1, msg::PointField::FLOAT32);
    else
	modifier.setPointCloud2Fields(3,
				      "x", 1, msg::PointField::FLOAT32,
				      "y", 1, msg::PointField::FLOAT32,
				      "z", 1, msg::PointField::FLOAT32);

    cv::Mat_<float>	K(3, 3);
    std::copy_n(std::begin(camera_info.k), 9, K.begin());
    cv::Mat_<float>	D(1, 4);
    std::copy_n(std::begin(camera_info.d), 4, D.begin());

    cv::Mat_<cv::Point2f>	uv(depth.width, 1), xy(depth.width, 1);
    for (uint32_t u = 0; u < depth.width; ++u)
	uv(u).x = u;

    for (uint32_t v = 0; v < depth.height; ++v)
    {
	PointCloud2Iterator<float>	xyz(cloud, "x");
	xyz += v * cloud.width;

	for (uint32_t u = 0; u < depth.width; ++u)
	    uv(u).y = v;

	cv::undistortPoints(uv, xy, K, D);

	auto	p = ptr<T>(depth, v);
	for (uint32_t u = 0; u < depth.width; ++u)
	{
	    const auto	d = meters(*p++);

	    if (float(d) == 0.0f)
	    {
	    	xyz[0] = xyz[1] = xyz[2]
	    	       = std::numeric_limits<float>::quiet_NaN();
	    }
	    else
	    {
	    	xyz[0] = xy(u).x * d;
	    	xyz[1] = xy(u).y * d;
	    	xyz[2] = d;
	    }

	    ++xyz;
	}
    }

    return cloud;
}

template <class T> sensor_msgs::msg::PointCloud2
create_pointcloud(const sensor_msgs::msg::CameraInfo& camera_info,
		  const sensor_msgs::msg::Image& depth,
		  const sensor_msgs::msg::Image& normal)
{
    using namespace	sensor_msgs;

    msg::PointCloud2	cloud;
    cloud.header	= depth.header;
    cloud.height	= depth.height;
    cloud.width		= depth.width;
    cloud.is_bigendian	= false;
    cloud.is_dense	= false;

    PointCloud2Modifier	modifier(cloud);
    modifier.setPointCloud2Fields(6,
				  "x",	      1, msg::PointField::FLOAT32,
				  "y",	      1, msg::PointField::FLOAT32,
				  "z",	      1, msg::PointField::FLOAT32,
				  "normal_x", 1, msg::PointField::FLOAT32,
				  "normal_y", 1, msg::PointField::FLOAT32,
				  "normal_z", 1, msg::PointField::FLOAT32);

    cv::Mat_<float>	K(3, 3);
    std::copy_n(std::begin(camera_info.k), 9, K.begin());
    cv::Mat_<float>	D(1, 4);
    std::copy_n(std::begin(camera_info.d), 4, D.begin());

    cv::Mat_<cv::Point2f>	uv(depth.width, 1), xy(depth.width, 1);
    for (uint32_t u = 0; u < depth.width; ++u)
	uv(u).x = u;

    for (uint32_t v = 0; v < depth.height; ++v)
    {
	PointCloud2Iterator<float>	xyzn(cloud, "x");
	xyzn += v * cloud.width;

	for (uint32_t u = 0; u < depth.width; ++u)
	    uv(u).y = v;

	cv::undistortPoints(uv, xy, K, D);

	using	vec3_t = float[3];
	auto	p = ptr<T>(depth, v);
	auto	n = ptr<vec3_t>(normal, v);
	for (uint32_t u = 0; u < depth.width; ++u)
	{
	    const auto	d = meters(*p++);

	    if (float(d) == 0.0f)
	    {
	    	xyzn[0] = xyzn[1] = xyzn[2] = xyzn[3] = xyzn[4] = xyzn[5]
	    	       = std::numeric_limits<float>::quiet_NaN();
	    }
	    else
	    {
	    	xyzn[0] = xy(u).x * d;
	    	xyzn[1] = xy(u).y * d;
	    	xyzn[2] = d;
		xyzn[3] = (*n)[0];
		xyzn[4] = (*n)[1];
		xyzn[5] = (*n)[2];
	    }

	    ++xyzn;
	    ++n;
	}
    }

    return cloud;
}

inline sensor_msgs::msg::PointCloud2
create_empty_pointcloud(const rclcpp::Time& stamp, const std::string& frame)
{
    using namespace	sensor_msgs;

    msg::PointCloud2	cloud;
    cloud.header.stamp	  = stamp;
    cloud.header.frame_id = frame;
    cloud.height	  = 1;
    cloud.width		  = 0;
    cloud.is_bigendian	  = false;
    cloud.is_dense	  = true;

    PointCloud2Modifier	modifier(cloud);
    modifier.setPointCloud2Fields(3,
				  "x", 1, msg::PointField::FLOAT32,
				  "y", 1, msg::PointField::FLOAT32,
				  "z", 1, msg::PointField::FLOAT32);

    return cloud;
}

template <class T> sensor_msgs::msg::PointCloud2&
add_rgb_to_pointcloud(sensor_msgs::msg::PointCloud2& cloud,
		      const sensor_msgs::msg::Image& image)
{
    using namespace	sensor_msgs;

    if (cloud.height != image.height || cloud.width != image.width)
	throw std::runtime_error("add_rgb_to_pointcloud(): cloud and image with different sizes!");

    PointCloud2Iterator<uint8_t>	bgr(cloud, "rgb");
    for (uint32_t v = 0; v < cloud.height; ++v)
    {
	auto	p = ptr<T>(image, v);

	for (uint32_t u = 0; u < cloud.width; ++u)
	{
	    set_color<T>(bgr, *p);
	    ++bgr;
	    ++p;
	}
    }

    return cloud;
}

template <class T> sensor_msgs::msg::PointCloud2&
add_rgb_to_pointcloud(sensor_msgs::msg::PointCloud2& cloud, const T& color)
{
    using namespace	sensor_msgs;

    PointCloud2Iterator<uint8_t>	bgr(cloud, "rgb");
    for (uint32_t v = 0; v < cloud.height; ++v)
    {
	for (uint32_t u = 0; u < cloud.width; ++u)
	{
	    set_color<T>(bgr, color);
	    ++bgr;
	}
    }

    return cloud;
}

template <class T> sensor_msgs::msg::PointCloud2&
add_3field_to_pointcloud(sensor_msgs::msg::PointCloud2& cloud,
			 const sensor_msgs::msg::Image& image,
			 const std::string& field_name)
{
    using namespace	sensor_msgs;

    if (cloud.height != image.height || cloud.width != image.width)
	throw std::runtime_error("add_3field_to_pointcloud(): cloud and image with different sizes!");

    PointCloud2Iterator<T*>	field(cloud, field_name);
    for (uint32_t v = 0; v < cloud.height; ++v)
    {
	auto	p = ptr<T>(image, v);

	for (uint32_t u = 0; u < cloud.width; ++u)
	{
	    field[0] = *p++;
	    field[1] = *p++;
	    field[2] = *p++;
	    ++field;
	}
    }

    return cloud;
}

/*
 *  Emit sensor messages in YAML format
 */
inline YAML::Emitter&
operator <<(YAML::Emitter& emitter,
	    const sensor_msgs::msg::RegionOfInterest& roi)
{
    return emitter << YAML::BeginMap
		   << YAML::Key   << "x_offset" << YAML::Value << roi.x_offset
		   << YAML::Key   << "y_offset" << YAML::Value << roi.y_offset
		   << YAML::Key   << "height"   << YAML::Value << roi.height
		   << YAML::Key   << "width"    << YAML::Value << roi.width
		   << YAML::Key   << "do_rectify"
		   << YAML::Value << roi.do_rectify;
}

inline YAML::Emitter&
operator <<(YAML::Emitter& emitter,
	    const sensor_msgs::msg::CameraInfo& camera_info)
{
    emitter << YAML::BeginMap
	    << YAML::Key   << "header" << YAML::Value << camera_info.header
	    << YAML::Key   << "height" << YAML::Value << camera_info.height
	    << YAML::Key   << "width"  << YAML::Value << camera_info.width
	    << YAML::Key   << "distortion_model"
	    << YAML::Value << camera_info.distortion_model;

    emitter << YAML::Key << "D" << YAML::Value << YAML::BeginSeq;
    for (const auto& d : camera_info.d)
	emitter << d;
    emitter << YAML::EndSeq;

    emitter << YAML::Key << "K" << YAML::Value << YAML::BeginSeq;
    for (const auto& k : camera_info.k)
	emitter << k;
    emitter << YAML::EndSeq;

    emitter << YAML::Key << "R" << YAML::Value << YAML::BeginSeq;
    for (const auto& r : camera_info.r)
	emitter << r;
    emitter << YAML::EndSeq;

    emitter << YAML::Key << "P" << YAML::Value << YAML::BeginSeq;
    for (const auto& p : camera_info.p)
	emitter << p;
    emitter << YAML::EndSeq;

    return emitter << YAML::Key   << "binning_x"
		   << YAML::Value << camera_info.binning_x
		   << YAML::Key   << "binning_y"
		   << YAML::Value << camera_info.binning_y
		   << YAML::Key   << "roi"
		   << YAML::Value << camera_info.roi
		   << YAML::EndMap;
}
}	// namespace aist_utility

