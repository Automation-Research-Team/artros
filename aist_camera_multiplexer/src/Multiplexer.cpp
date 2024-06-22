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
#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <ddynamic_reconfigure2/ddynamic_reconfigure2.hpp>

namespace aist_camera_multiplexer
{
/************************************************************************
*  class Multiplexer::Subscribers					*
************************************************************************/
class Multiplexer : public rclcpp::Node
{
  private:
    using image_t	= sensor_msgs::msg::Image;
    using image_p	= image_t::ConstSharedPtr;
    using cloud_t	= sensor_msgs::msg::PointCloud2;
    using cloud_p	= cloud_t::UniquePtr;
    using camera_info_t	= sensor_msgs::msg::CameraInfo;
    using camera_info_p	= camera_info_t::UniquePtr;

    class Subscribers
    {
      public:
	Subscribers(Multiplexer* multiplexer,
		    const std::string& camera_name)			;

	const std::string&	camera_name()			const	;

      private:
	const std::string				     _camera_name;
	image_transport::Subscriber			     _image_sub;
	image_transport::Subscriber			     _depth_sub;
	image_transport::Subscriber			     _normal_sub;
	const rclcpp::Subscription<cloud_t>::SharedPtr	     _cloud_sub;
	const rclcpp::Subscription<camera_info_t>::SharedPtr _camera_info_sub;
    };
    
    using subscribers_cp	 = std::shared_ptr<const Subscribers>;
    using ddynamic_reconfigure_t = ddynamic_reconfigure2::DDynamicReconfigure;

  public:
    Multiplexer(const rclcpp::NodeOptions& options)			;

  private:
    std::string	node_name()		const	{ return get_name(); }
    template <class T>
    T		declare_read_only_parameter(const std::string& name,
					    const T& default_value)	;
    size_t	ncameras()					const	;
    void	activate_camera(const std::string& camera_name)		;

  private:
    std::vector<subscribers_cp>				_subscribers;
    size_t						_active_camera_number;

    ddynamic_reconfigure_t				_ddr;

    image_transport::ImageTransport			_it;
    const image_transport::Publisher			_image_pub;
    const image_transport::Publisher			_depth_pub;
    const image_transport::Publisher			_normal_pub;
    const rclcpp::Publisher<cloud_t>::SharedPtr		_cloud_pub;
    const rclcpp::Publisher<camera_info_t>::SharedPtr	_camera_info_pub;
};

Multiplexer::Subscribers::Subscribers(Multiplexer* multiplexer,
				      const std::string& camera_name)
    :_camera_name(camera_name),
     _image_sub(multiplexer->_it.subscribe(
		    _camera_name + "/image", 1,
		    [multiplexer, n=multiplexer->ncameras()](image_p image)
		    {
			if (n == multiplexer->_active_camera_number)
			    multiplexer->_image_pub.publish(image);
		    })),
     _depth_sub(multiplexer->_it.subscribe(
		    _camera_name + "/depth", 1,
		    [multiplexer, n=multiplexer->ncameras()](image_p depth)
		    {
			if (n == multiplexer->_active_camera_number)
			    multiplexer->_depth_pub.publish(depth);
		    })),
     _normal_sub(multiplexer->_it.subscribe(
		     _camera_name + "/normal", 1,
		    [multiplexer, n=multiplexer->ncameras()](image_p normal)
		    {
			if (n == multiplexer->_active_camera_number)
			    multiplexer->_normal_pub.publish(normal);
		    })),
     _cloud_sub(multiplexer->create_subscription<cloud_t>(
		    _camera_name + "/pointcloud", 1,
		    [multiplexer, n=multiplexer->ncameras()](cloud_p cloud)
		    {
			if (n == multiplexer->_active_camera_number)
			    multiplexer->_cloud_pub->publish(std::move(cloud));
		    })),
     _camera_info_sub(multiplexer->create_subscription<camera_info_t>(
			  _camera_name + "/camera_info", 1,
			  [multiplexer, n=multiplexer->ncameras()]
			  (camera_info_p camera_info)
			  {
			      if (n == multiplexer->_active_camera_number)
				  multiplexer->_camera_info_pub
					     ->publish(std::move(camera_info));
			  }))
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
Multiplexer::Multiplexer(const rclcpp::NodeOptions& options)
    :rclcpp::Node("camera_multiplexer", options),
     _subscribers(),
     _active_camera_number(0),
     _ddr(rclcpp::Node::SharedPtr(this)),
     _it(rclcpp::Node::SharedPtr(this)),
     _image_pub( _it.advertise(node_name() + "/image",  1)),
     _depth_pub( _it.advertise(node_name() + "/depth",  1)),
     _normal_pub(_it.advertise(node_name() + "/normal", 1)),
     _cloud_pub(create_publisher<cloud_t>(node_name() + "/pointcloud", 1)),
     _camera_info_pub(create_publisher<camera_info_t>(
			  node_name() + "/camera_info", 1))
{
    const auto	camera_names = declare_read_only_parameter<
				std::vector<std::string> >("camera_names", {});
    if (camera_names.empty())
    {
	RCLCPP_ERROR_STREAM(get_logger(), "No camera names specified.");
	throw;
    }

    std::map<std::string, std::string>	enum_cameras;
    for (const auto& camera_name : camera_names)
    {
	enum_cameras[camera_name] = camera_name;
	_subscribers.emplace_back(new Subscribers(this, camera_name));
	RCLCPP_INFO_STREAM(get_logger(),
			   "Subscribe camera[" << camera_name << ']');
    }

    _ddr.registerEnumVariable<std::string>(
	"active_camera", _subscribers[_active_camera_number]->camera_name(),
	std::bind(&Multiplexer::activate_camera, this, std::placeholders::_1),
	"Currently active camera", enum_cameras);
}

template <class T> T
Multiplexer::declare_read_only_parameter(const std::string& name,
					 const T& default_value)
{
    return declare_parameter(
		name, default_value,
		ddynamic_reconfigure2::read_only_param_desc<T>(name));
}

size_t
Multiplexer::ncameras() const
{
    return _subscribers.size();
}

void
Multiplexer::activate_camera(const std::string& camera_name)
{
    for (size_t i = 0; i < ncameras(); ++i)
	if (_subscribers[i]->camera_name() == camera_name)
	{
	    _active_camera_number = i;
	    RCLCPP_INFO_STREAM(get_logger(),
			       "Activate camera[" << camera_name << ']');
	    return;
	}

    RCLCPP_ERROR_STREAM(get_logger(), "Unknown camera[" << camera_name << ']');
}

}	// namespace aist_camera_multiplexer

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(aist_camera_multiplexer::Multiplexer)
