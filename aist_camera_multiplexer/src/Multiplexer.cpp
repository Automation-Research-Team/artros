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
 *  \file	Multiplexer.cpp
 *  \author	Toshio Ueshiba
 *  \brief	ROS node for selecting one camera from multiple cameras
 */
#include "Multiplexer.h"

namespace aist_camera_multiplexer
{
/************************************************************************
*  class Multiplexer::Subscribers					*
************************************************************************/
Multiplexer::Subscribers::Subscribers(ros::NodeHandle& nh,
				      Multiplexer* multiplexer,
				      const std::string& camera_name)
    :_camera_name(camera_name),
     _image_sub(multiplexer->_it.subscribe(
		    _camera_name + "/image", 1,
		    boost::bind(&Multiplexer::image_cb,
				multiplexer, _1, multiplexer->ncameras()))),
     _depth_sub(multiplexer->_it.subscribe(
		    _camera_name + "/depth", 1,
		    boost::bind(&Multiplexer::depth_cb,
				multiplexer, _1, multiplexer->ncameras()))),
     _normal_sub(multiplexer->_it.subscribe(
		     _camera_name + "/normal", 1,
		     boost::bind(&Multiplexer::normal_cb,
				 multiplexer, _1, multiplexer->ncameras()))),
     _cloud_sub(nh.subscribe<cloud_t>(
			  _camera_name + "/pointcloud", 1,
			  boost::bind(&Multiplexer::cloud_cb,
				      multiplexer, _1,
				      multiplexer->ncameras()))),
     _camera_info_sub(nh.subscribe<camera_info_t>(
			  _camera_name + "/camera_info", 1,
			  boost::bind(&Multiplexer::camera_info_cb,
				      multiplexer, _1,
				      multiplexer->ncameras())))
{
}

const std::string&
Multiplexer::Subscribers::camera_name() const
{
    return _camera_name;
}

/************************************************************************
*  class Multiplexer							*
************************************************************************/
Multiplexer::Multiplexer(ros::NodeHandle& nh)
    :_subscribers(),
     _active_camera_number(0),
     _ddr(nh),
     _it(nh),
     _image_pub( _it.advertise("image",  1)),
     _depth_pub( _it.advertise("depth",  1)),
     _normal_pub(_it.advertise("normal", 1)),
     _cloud_pub(nh.advertise<cloud_t>("pointcloud", 1)),
     _camera_info_pub(nh.advertise<camera_info_t>("camera_info", 1))
{
    std::vector<std::string>	camera_names;
    if (!nh.getParam("camera_names", camera_names))
    {
	ROS_ERROR_STREAM("(Multiplexer) no camera names specified.");
	return;
    }

    std::map<std::string, std::string>	enum_cameras;
    for (const auto& camera_name : camera_names)
    {
	enum_cameras[camera_name] = camera_name;
	_subscribers.emplace_back(new Subscribers(nh, this, camera_name));
	ROS_INFO_STREAM("(Multiplexer) subscribe camera["
			<< camera_name << ']');
    }

    _ddr.registerEnumVariable<std::string>(
	"active_camera", _subscribers[_active_camera_number]->camera_name(),
	boost::bind(&Multiplexer::activate_camera, this, _1),
	"Currently active camera", enum_cameras);
    _ddr.publishServicesTopicsAndUpdateConfigData();
}

int
Multiplexer::ncameras() const
{
    return _subscribers.size();
}

void
Multiplexer::activate_camera(const std::string& camera_name)
{
    for (int i = 0; i < ncameras(); ++i)
	if (_subscribers[i]->camera_name() == camera_name)
	{
	    _active_camera_number = i;
	    ROS_INFO_STREAM("(Multiplexer) activate camera["
			    << camera_name << ']');
	    return;
	}

    ROS_ERROR_STREAM("(Multiplexer) unknown camera[" << camera_name << ']');
}

void
Multiplexer::camera_info_cb(const camera_info_cp& camera_info,
			    int camera_number) const
{
    if (camera_number == _active_camera_number)
	_camera_info_pub.publish(camera_info);
}

void
Multiplexer::image_cb(const image_cp& image, int camera_number) const
{
    if (camera_number == _active_camera_number)
	_image_pub.publish(image);
}

void
Multiplexer::depth_cb(const image_cp& depth, int camera_number) const
{
    if (camera_number == _active_camera_number)
	_depth_pub.publish(depth);
}

void
Multiplexer::normal_cb(const image_cp& normal, int camera_number) const
{
    if (camera_number == _active_camera_number)
	_normal_pub.publish(normal);
}

void
Multiplexer::cloud_cb(const cloud_cp& cloud, int camera_number) const
{
    if (camera_number == _active_camera_number)
	_cloud_pub.publish(cloud);
}

}	// namespace aist_camera_multiplexer
