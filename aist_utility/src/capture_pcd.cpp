/*!
 *  \file	capture_pcd.cpp
 *  \author	Toshio Ueshiba
 */
#include <cstdlib>	// for getenv()
#include <sys/stat.h>	// for mkdir()
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc.hpp>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf/transform_listener.h>
#include <std_srvs/Trigger.h>
#include <aist_utility/sensor_msgs.h>

namespace aist_utility
{
/************************************************************************
*   class PCDCapturer							*
************************************************************************/
class PCDCapturer
{
  private:
    using image_t	= sensor_msgs::Image;
    using image_cp	= sensor_msgs::ImageConstPtr;
    using camera_info_t	= sensor_msgs::CameraInfo;
    using camera_info_cp= sensor_msgs::CameraInfoConstPtr;
    using cloud_t	= sensor_msgs::PointCloud2;
    using cloud_cp	= sensor_msgs::PointCloud2ConstPtr;
    using sync_t	= message_filters::TimeSynchronizer<image_t, image_t,
							    camera_info_t>;
    using pcl_point_t	= pcl::PointXYZRGB;
    using pcl_cloud_t	= pcl::PointCloud<pcl_point_t>;
    using pcl_cloud_p	= pcl_cloud_t::Ptr;

    struct rgb_t	{ uint8_t	r, g, b; };
    
  public:
		PCDCapturer(ros::NodeHandle& nh)			;

  private:
    void	camera_cb(const image_cp& color, const image_cp& depth,
			  const camera_info_cp& camera_info)		;
    bool	save_cloud_cb(std_srvs::Trigger::Request&  req,
			      std_srvs::Trigger::Response& res)		;
    static
    std::string	open_dir()						;

  private:
    image_transport::ImageTransport		_it;
    image_transport::SubscriberFilter		_color_sub;
    image_transport::SubscriberFilter		_depth_sub;
    message_filters::Subscriber<camera_info_t>	_camera_info_sub;
    sync_t					_sync;
    const ros::Publisher			_cloud_pub;
    const ros::ServiceServer			_save_cloud;

    const tf::TransformListener			_listener;
    const std::string				_cloud_frame;
    const bool					_save_as_binary;
    size_t					_file_num;
    cloud_t					_cloud;
};

PCDCapturer::PCDCapturer(ros::NodeHandle& nh)
    :_it(nh),
     _color_sub(_it, "/color", 1),
     _depth_sub(_it, "/depth", 1),
     _camera_info_sub(nh, "/camera_info", 1),
     _sync(_color_sub, _depth_sub, _camera_info_sub, 1),
     _cloud_pub(nh.advertise<sensor_msgs::PointCloud2>("pointcloud", 1)),
     _save_cloud(nh.advertiseService("save_cloud",
				     &PCDCapturer::save_cloud_cb, this)),
     _listener(),
     _cloud_frame(nh.param<std::string>("cloud_frame", "base_link")),
     _save_as_binary(nh.param<bool>("save_as_binary", false)),
     _file_num(0),
     _cloud()
{
  // Register callback for subscribing synched color, depth and camera_info.
    _sync.registerCallback(&PCDCapturer::camera_cb, this);
}

void
PCDCapturer::camera_cb(const image_cp& color, const image_cp& depth,
		       const camera_info_cp& camera_info)
{
    if (depth->encoding == sensor_msgs::image_encodings::TYPE_16UC1)
	_cloud = aist_utility::create_pointcloud<uint16_t>(*camera_info,
							   *depth, true);
    else if (depth->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
	_cloud = aist_utility::create_pointcloud<float>(*camera_info,
							*depth, true);
    else
    {
	ROS_ERROR_STREAM("Unknown depth type[" << depth->encoding << ']');
	return;
    }
    
    if (color->encoding == sensor_msgs::image_encodings::MONO8)
	aist_utility::add_rgb_to_pointcloud<uint8_t>(_cloud, *color);
    else if (color->encoding == sensor_msgs::image_encodings::RGB8)
	aist_utility::add_rgb_to_pointcloud<rgb_t>(_cloud, *color);
    else
    {
	ROS_ERROR_STREAM("Unknown color encoding[" << color->encoding << ']');
	return;
    }

    _cloud_pub.publish(_cloud);
}

bool
PCDCapturer::save_cloud_cb(std_srvs::Trigger::Request&  req,
			   std_srvs::Trigger::Response& res)
{
    res.success = false;
    
  // Create PCL cloud from cloud of sensor_msgs::PointCloud2 type.
    pcl_cloud_p	pcl_cloud(new pcl_cloud_t);
    pcl::fromROSMsg(_cloud, *pcl_cloud);

  // Transform PCL cloud to _cloud_frame.
    if (!pcl_ros::transformPointCloud(_cloud_frame, *pcl_cloud, *pcl_cloud,
				      _listener))
    {
	ROS_ERROR_STREAM("(capture_pcd) Failed to transform cloud from "
			 << _cloud.header.frame_id << " to " << _cloud_frame);
	return true;
    }

  // Save PCL cloud as a PCD file.
    const auto	file_path = open_dir() + "/capture_pcd-"
				       + std::to_string(_file_num) + ".pcd";
    
    if (pcl::io::savePCDFile<pcl_point_t>(file_path, *pcl_cloud,
					  _save_as_binary) < 0)
    {
	ROS_ERROR_STREAM("(capture_pcd) Failed to save point cloud to file["
			 << file_path << ']');
	return true;
    }
    
    ++_file_num;
    res.success = true;

    ROS_INFO_STREAM("(capture_pcd) Saved point cloud to file["
		    << file_path << ']');

    return true;
}

std::string
PCDCapturer::open_dir()
{
    const auto	home = getenv("HOME");
    if (!home)
	throw std::runtime_error("Environment variable[HOME] is not set.");

    const auto	dir_name = home + std::string("/.ros")
				+ ros::this_node::getNamespace();
    struct stat	buf;
    if (stat(dir_name.c_str(), &buf) && mkdir(dir_name.c_str(), S_IRWXU))
	throw std::runtime_error("Cannot create " + dir_name + ": "
						  + strerror(errno));

    return dir_name;
}
}

/************************************************************************
*   main(): required only for building conventional(non-nodelet) nodes	*
************************************************************************/
int
main(int argc, char* argv[])
{
    ros::init(argc, argv, "capture_pcd");

    try
    {
	ros::NodeHandle			nh("~");
	aist_utility::PCDCapturer	capturer(nh);

	ros::spin();
    }
    catch (const std::exception& err)
    {
	std::cerr << err.what() << std::endl;
	return 1;
    }

    return 0;
}
