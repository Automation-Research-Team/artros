/*!
* \file		Detector.h
* \author	Toshio UESHIBA
* \brief	ARuCo marker detector using both intensity and depth images
*/
#include <iostream>
#include <limits>
#include <cstdint>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <ddynamic_reconfigure/ddynamic_reconfigure.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <aruco/aruco.h>

namespace aist_aruco_ros
{
/************************************************************************
*  class Detector							*
************************************************************************/
class Detector
{
  private:
    using camera_info_t		= sensor_msgs::CameraInfo;
    using camera_info_p		= sensor_msgs::CameraInfoConstPtr;
    using image_t		= sensor_msgs::Image;
    using image_p		= sensor_msgs::ImageConstPtr;
    using cloud_t		= sensor_msgs::PointCloud2;
    using cloud_p		= sensor_msgs::PointCloud2ConstPtr;
    using depth_sync_t		= message_filters::TimeSynchronizer<
					image_t, image_t, camera_info_t>;
    using cloud_sync_t		= message_filters::TimeSynchronizer<
				cloud_t, camera_info_t>;
    using mdetector_t		= aruco::MarkerDetector;
    using mparams_t		= mdetector_t::Params;
    using marker_info_t		= aruco::Marker3DInfo;
    using marker_map_t		= aruco::MarkerMap;
    using point3_t		= cv::Vec<float, 3>;
    using ddynamic_reconfigure_t= ddynamic_reconfigure::DDynamicReconfigure;

    struct rgb_t		{ uint8_t r, g, b; };

  public:
		Detector(ros::NodeHandle& nh)				;

  private:
    void	set_detection_mode(int mode)				;
    void	set_min_marker_size(double size)			;
    void	set_dictionary(int dict_type)				;
    void	detect_marker_from_depth_cb(
		    const image_p&	 image_msg,
		    const image_p&	 depth_msg,
		    const camera_info_p& camera_info_msg)		;
    void	detect_marker_from_cloud_cb(
		    const cloud_p&	 cloud_msg,
		    const camera_info_p& camera_info_msg)		;
    template <class MSG>
    void	detect_marker(const MSG& msg,
			      const camera_info_t& camera_info_msg,
			      cv::Mat& image)				;
    static void	publish_image(const std_msgs::Header& header,
			      const cv::Mat& image,
			      const image_transport::Publisher& pub)	;
    template <class ITER>
    void	publish_transform(ITER begin, ITER end,
				  const std_msgs::Header& header,
				  const std::string& marker_frame,
				  bool publish_pose)			;
    template <class MSG> std::vector<point3_t>
		get_marker_corners(const aruco::Marker& marker,
				   const MSG& msg,
				   cv::Mat& image)		const	;
    template <class T> cv::Vec<T, 3>
		view_vector(T u, T v)				const	;
    template <class T> cv::Vec<T, 3>
		at(const image_t& depth_msg, int u, int v)	const	;
    template <class T> cv::Vec<T, 3>
		at(const cloud_t& cloud_msg, int u, int v)	const	;
    template <class T, class MSG> cv::Vec<T, 3>
		at(const MSG& msg, T u, T v)			const	;

  private:
  // transformation stuff
    tf2_ros::TransformBroadcaster		_broadcaster;
    std::string					_marker_frame;

  // input camera_info/image stuff
    image_transport::ImageTransport		_it;
    image_transport::SubscriberFilter		_image_sub;
    image_transport::SubscriberFilter		_depth_sub;
    message_filters::Subscriber<cloud_t>	_cloud_sub;
    message_filters::Subscriber<camera_info_t>	_camera_info_sub;
    depth_sync_t				_depth_sync;
    cloud_sync_t				_cloud_sync;

  // camera_info stuff
    aruco::CameraParameters			_camParam;
    bool					_useRectifiedImages;
    tf2::Transform				_rightToLeft;

  // output stuff
    const image_transport::Publisher		_result_pub;
    const image_transport::Publisher		_debug_pub;
    const ros::Publisher			_pose_pub;

    ddynamic_reconfigure_t			_ddr;

    mdetector_t					_marker_detector;
    marker_map_t				_marker_map;
    float					_marker_size;
    bool					_useSimilarity;
    double					_planarityTolerance;
};

}	// namespace aist_aruco_ros
