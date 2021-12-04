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
#ifndef AIST_UTILITY_SENSOR_MSGS_H
#define AIST_UTILITY_SENSOR_MSGS_H

#include <array>
#include <algorithm>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>

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
ptr(sensor_msgs::Image& image, int v)
{
    return reinterpret_cast<T*>(image.data.data() + v*image.step);
}

//! Get const pointer to the leftmost pixel of v-th scanline of the image.
template <class T> inline const T*
ptr(const sensor_msgs::Image& image, int v)
{
    return reinterpret_cast<const T*>(image.data.data() + v*image.step);
}

//! Convert depth image to sequence of 3D points.
template <class T, class ITER, class UNIT> ITER
depth_to_points(const sensor_msgs::CameraInfo& camera_info,
		const sensor_msgs::Image& depth, ITER out, UNIT unit)
{
    cv::Mat_<float>	K(3, 3);
    std::copy_n(std::begin(camera_info.K), 9, K.begin());
    cv::Mat_<float>	D(1, 4);
    std::copy_n(std::begin(camera_info.D), 4, D.begin());

    cv::Mat_<cv::Point2f>	uv(depth.width, 1), xy(depth.width, 1);
    for (int u = 0; u < depth.width; ++u)
	uv(u).x = u;

    for (int v = 0; v < depth.height; ++v)
    {
	for (int u = 0; u < depth.width; ++u)
	    uv(u).y = v;

	cv::undistortPoints(uv, xy, K, D);

	auto	p = ptr<T>(depth, v);
	for (int u = 0; u < depth.width; ++u)
	{
	    const auto	d = unit(*p++);		// meters or milimeters
	    *out++ = {xy(u).x * d, xy(u).y * d, d};
	}
    }

    return out;
}

//! Extract 3D points from pointcloud.
template <class T=float, class ITER, class UNIT> ITER
cloud_to_points(const sensor_msgs::PointCloud2& cloud, ITER out, UNIT unit)
{
    sensor_msgs::PointCloud2ConstIterator<T>	xyz(cloud, "x");

    for (int v = 0; v < cloud.height; ++v)
	for (int u = 0; u < cloud.width; ++u)
	{
	    *out++ = {unit(xyz[0]), unit(xyz[1]), unit(xyz[2])};
	    ++xyz;
	}

    return out;
}

template <class IN> sensor_msgs::PointCloud2
create_pointcloud(IN in, IN ie,
		  const ros::Time& stamp, const std::string& frame)
{
    using namespace	sensor_msgs;

    PointCloud2	cloud;
    cloud.is_bigendian	  = false;
    cloud.is_dense	  = true;
    cloud.header.stamp	  = stamp;
    cloud.header.frame_id = frame;
    cloud.height	  = 1;
    cloud.width		  = std::distance(in, ie);

    PointCloud2Modifier	modifier(cloud);
    modifier.setPointCloud2Fields(3,
				  "x", 1, PointField::FLOAT32,
				  "y", 1, PointField::FLOAT32,
				  "z", 1, PointField::FLOAT32);
    modifier.resize(cloud.width);
    cloud.row_step = cloud.width * cloud.point_step;

    PointCloud2Iterator<float>	out(cloud, "x");
    for (; in != ie; ++in, ++out)
    {
	const auto&	xyz = *in;
	out[0] = xyz(0);
	out[1] = xyz(1);
	out[2] = xyz(2);
    }

    return cloud;
}

template <class T> sensor_msgs::PointCloud2
create_pointcloud(const sensor_msgs::CameraInfo& camera_info,
		  const sensor_msgs::Image& depth, bool with_rgb=false)
{
    using namespace	sensor_msgs;

    PointCloud2	cloud;
    cloud.is_bigendian	= false;
    cloud.is_dense	= false;

    PointCloud2Modifier	modifier(cloud);
    if (with_rgb)
	modifier.setPointCloud2Fields(4,
				      "x",   1, PointField::FLOAT32,
				      "y",   1, PointField::FLOAT32,
				      "z",   1, PointField::FLOAT32,
				      "rgb", 1, PointField::FLOAT32);
    else
	modifier.setPointCloud2Fields(3,
				      "x", 1, PointField::FLOAT32,
				      "y", 1, PointField::FLOAT32,
				      "z", 1, PointField::FLOAT32);
    modifier.resize(depth.height * depth.width);

    cloud.header	= depth.header;
    cloud.height	= depth.height;
    cloud.width		= depth.width;
    cloud.row_step	= cloud.width * cloud.point_step;

    cv::Mat_<float>	K(3, 3);
    std::copy_n(std::begin(camera_info.K), 9, K.begin());
    cv::Mat_<float>	D(1, 4);
    std::copy_n(std::begin(camera_info.D), 4, D.begin());

    cv::Mat_<cv::Point2f>	uv(depth.width, 1), xy(depth.width, 1);
    for (int u = 0; u < depth.width; ++u)
	uv(u).x = u;

    for (int v = 0; v < depth.height; ++v)
    {
	PointCloud2Iterator<float>	xyz(cloud, "x");
	xyz += v * cloud.width;

	for (int u = 0; u < depth.width; ++u)
	    uv(u).y = v;

	cv::undistortPoints(uv, xy, K, D);

	auto	p = ptr<T>(depth, v);
	for (int u = 0; u < depth.width; ++u)
	{
	    const auto	d = meters<T>(*p++);

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

template <class T> sensor_msgs::PointCloud2
create_pointcloud(const sensor_msgs::CameraInfo& camera_info,
		  const sensor_msgs::Image& depth,
		  const sensor_msgs::Image& normal)
{
    using namespace	sensor_msgs;

    PointCloud2	cloud;
    cloud.is_bigendian	= false;
    cloud.is_dense	= false;

    PointCloud2Modifier	modifier(cloud);
    modifier.setPointCloud2Fields(6,
				  "x",	      1, PointField::FLOAT32,
				  "y",	      1, PointField::FLOAT32,
				  "z",	      1, PointField::FLOAT32,
				  "normal_x", 1, PointField::FLOAT32,
				  "normal_y", 1, PointField::FLOAT32,
				  "normal_z", 1, PointField::FLOAT32);
    modifier.resize(depth.height * depth.width);

    cloud.header	= depth.header;
    cloud.height	= depth.height;
    cloud.width		= depth.width;
    cloud.row_step	= cloud.width * cloud.point_step;

    cv::Mat_<float>	K(3, 3);
    std::copy_n(std::begin(camera_info.K), 9, K.begin());
    cv::Mat_<float>	D(1, 4);
    std::copy_n(std::begin(camera_info.D), 4, D.begin());

    cv::Mat_<cv::Point2f>	uv(depth.width, 1), xy(depth.width, 1);
    for (int u = 0; u < depth.width; ++u)
	uv(u).x = u;

    for (int v = 0; v < depth.height; ++v)
    {
	PointCloud2Iterator<float>	xyzn(cloud, "x");
	xyzn += v * cloud.width;

	for (int u = 0; u < depth.width; ++u)
	    uv(u).y = v;

	cv::undistortPoints(uv, xy, K, D);

	using	vec3_t = float[3];
	auto	p = ptr<T>(depth, v);
	auto	n = ptr<vec3_t>(normal, v);
	for (int u = 0; u < depth.width; ++u)
	{
	    const auto	d = meters<T>(*p++);

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

template <class T> sensor_msgs::PointCloud2&
add_rgb_to_pointcloud(sensor_msgs::PointCloud2& cloud,
		      const sensor_msgs::Image& image)
{
    using namespace	sensor_msgs;

    if (cloud.height != image.height || cloud.width != image.width)
	throw std::runtime_error("add_rgb_to_pointcloud(): cloud and image with different sizes!");
    
    PointCloud2Iterator<uint8_t>	rgb(cloud, "rgb");
    for (int v = 0; v < cloud.height; ++v)
    {
	auto	p = ptr<T>(image, v);

	for (int u = 0; u < cloud.width; ++u)
	{
	    rgb[0] = p->b;
	    rgb[1] = p->g;
	    rgb[2] = p->r;
	    ++rgb;
	    ++p;
	}
    }

    return cloud;
}

}	// namespace aist_utility
#endif	// !AIST_UTILITY_SENSOR_MSGS_H
