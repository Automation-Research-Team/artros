/*!
 *  \file	capture_pcd.cpp
 *  \author	Toshio Ueshiba
 */
#include <cstdlib>	// for getenv()
#include <sys/stat.h>	// for mkdir()
#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <image_transport/subscriber_filter.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <opencv2/imgproc.hpp>
#include <pcl_ros/transforms.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/transform_listener.h>
#include <std_srvs/srv/trigger.hpp>
#include <aist_utility/sensor_msgs.hpp>
#include <ddynamic_reconfigure2/ddynamic_reconfigure2.hpp>

namespace aist_utility
{
/************************************************************************
*   class PCDCapturer							*
************************************************************************/
class PCDCapturer : public rclcpp::Node
{
  private:
    using image_t	= sensor_msgs::msg::Image;
    using image_cp	= image_t::ConstSharedPtr;
    using camera_info_t	= sensor_msgs::msg::CameraInfo;
    using camera_info_cp= camera_info_t::ConstSharedPtr;
    using cloud_t	= sensor_msgs::msg::PointCloud2;
    using cloud_p	= cloud_t::UniquePtr;
    using sync_t	= message_filters::TimeSynchronizer<image_t, image_t,
							    camera_info_t>;
    using pcl_point_t	= pcl::PointXYZRGB;
    using pcl_cloud_t	= pcl::PointCloud<pcl_point_t>;
    using pcl_cloud_p	= pcl_cloud_t::Ptr;
    using trigger_t	= std_srvs::srv::Trigger;
    using trigger_srv_p	= rclcpp::Service<trigger_t>::SharedPtr;
    using trigger_req_p	= trigger_t::Request::SharedPtr;
    using trigger_res_p	= trigger_t::Response::SharedPtr;

    struct rgb_t	{ uint8_t	r, g, b; };

  public:
		PCDCapturer(const rclcpp::NodeOptions& options)		;

  private:
    std::string	node_name()			const	{ return get_name(); }
    void	camera_cb(const image_cp& color, const image_cp& depth,
			  const camera_info_cp& camera_info)		;
    bool	save_cloud_cb(const trigger_req_p, trigger_res_p res)	;
    std::string	open_dir()					const	;

  private:
    ddynamic_reconfigure2::DDynamicReconfigure	_ddr;
    image_transport::ImageTransport		_it;
    image_transport::SubscriberFilter		_color_sub;
    image_transport::SubscriberFilter		_depth_sub;
    message_filters::Subscriber<camera_info_t>	_camera_info_sub;
    sync_t					_sync;
    const rclcpp::Publisher<cloud_t>::SharedPtr	_cloud_pub;
    const rclcpp::Service<trigger_t>::SharedPtr	_save_cloud;

    tf2_ros::Buffer				_buffer;
    const tf2_ros::TransformListener		_listener;
    const std::string				_cloud_frame;
    const bool					_save_as_binary;
    size_t					_file_num;
    cloud_t					_cloud;
};

PCDCapturer::PCDCapturer(const rclcpp::NodeOptions& options)
    :rclcpp::Node("pcd_capturer", options),
     _ddr(rclcpp::Node::SharedPtr(this)),
     _it(rclcpp::Node::SharedPtr(this)),
     _color_sub(this, "/color", "raw"),
     _depth_sub(this, "/depth", "raw"),
     _camera_info_sub(this, "/camera_info"),
     _sync(_color_sub, _depth_sub, _camera_info_sub, 1),
     _cloud_pub(create_publisher<cloud_t>(node_name() + "/pointcloud", 1)),
     _save_cloud(create_service<trigger_t>(
		     node_name() + "/save_cloud",
		     std::bind(&PCDCapturer::save_cloud_cb, this,
			       std::placeholders::_1, std::placeholders::_2))),
     _buffer(get_clock()),
     _listener(_buffer),
     _cloud_frame(_ddr.declare_read_only_parameter<std::string>("cloud_frame",
								"base_link")),
     _save_as_binary(_ddr.declare_read_only_parameter<bool>("save_as_binary",
							    false)),
     _file_num(0),
     _cloud()
{
  // Register callback for subscribing synched color, depth and camera_info.
    _sync.registerCallback(&PCDCapturer::camera_cb, this);

    RCLCPP_INFO_STREAM(get_logger(), "started");
}

void
PCDCapturer::camera_cb(const image_cp& color, const image_cp& depth,
		       const camera_info_cp& camera_info)
{
    using namespace	sensor_msgs;

    cloud_p	cloud;

    if (depth->encoding == image_encodings::TYPE_16UC1)
	cloud = aist_utility::create_pointcloud<uint16_t>(*camera_info,
							  *depth, true);
    else if (depth->encoding == image_encodings::TYPE_32FC1)
	cloud = aist_utility::create_pointcloud<float>(*camera_info,
						       *depth, true);
    else
    {
	RCLCPP_ERROR_STREAM(get_logger(),
			    "Unknown depth type[" << depth->encoding << ']');
	return;
    }

    if (color->encoding == image_encodings::MONO8)
	aist_utility::add_rgb_to_pointcloud<uint8_t>(*cloud, *color);
    else if (color->encoding == image_encodings::RGB8)
	aist_utility::add_rgb_to_pointcloud<rgb_t>(*cloud, *color);
    else
    {
	RCLCPP_ERROR_STREAM(get_logger(), "Unknown color encoding["
			    << color->encoding << ']');
	return;
    }

    _cloud = *cloud;
    _cloud_pub->publish(std::move(cloud));
}

bool
PCDCapturer::save_cloud_cb(const trigger_req_p, trigger_res_p res)
{
    res->success = false;

  // Create PCL cloud from cloud of sensor_msgs::msg::PointCloud2 type.
    pcl_cloud_p	pcl_cloud(new pcl_cloud_t);
    pcl::fromROSMsg(_cloud, *pcl_cloud);

  // Transform PCL cloud to _cloud_frame.
    if (!pcl_ros::transformPointCloud(_cloud_frame, *pcl_cloud, *pcl_cloud,
				      _buffer))
    {
	RCLCPP_ERROR_STREAM(get_logger(),
			    "(capture_pcd) Failed to transform cloud from "
			    << _cloud.header.frame_id << " to "
			    << _cloud_frame);
	return true;
    }

  // Save PCL cloud as a PCD file.
    const auto	file_path = open_dir() + "capture_pcd-"
				       + std::to_string(_file_num) + ".pcd";

    if (pcl::io::savePCDFile<pcl_point_t>(file_path, *pcl_cloud,
					  _save_as_binary) < 0)
    {
	RCLCPP_ERROR_STREAM(get_logger(),
			    "(capture_pcd) Failed to save point cloud to file["
			    << file_path << ']');
	return true;
    }

    ++_file_num;
    res->success = true;

    RCLCPP_INFO_STREAM(get_logger(), "(capture_pcd) Saved point cloud to file["
		       << file_path << ']');

    return true;
}

std::string
PCDCapturer::open_dir() const
{
    const auto	home = getenv("HOME");
    if (!home)
	throw std::runtime_error("Environment variable[HOME] is not set.");

    const auto	dir_name = home + std::string("/.ros")
				+ get_namespace();
    struct stat	buf;
    if (stat(dir_name.c_str(), &buf) && mkdir(dir_name.c_str(), S_IRWXU))
	throw std::runtime_error("Cannot create " + dir_name + ": "
						  + strerror(errno));

    return dir_name;
}
}

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(aist_utility::PCDCapturer)
