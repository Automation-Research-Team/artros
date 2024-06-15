/*!
* \file		MultiDetector.cpp
* \author	Toshio UESHIBA
* \brief	ARuCo marker detector using intensity images
*/
#include <iostream>
#include <cstdint>
#include <any>

#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <image_transport/image_transport.hpp>
#include <image_transport/subscriber_filter.hpp>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <ddynamic_reconfigure2/ddynamic_reconfigure2.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <cv_bridge/cv_bridge.h>

#include <aist_utility/opencv.hpp>
#include <aist_utility/sensor_msgs.hpp>
#include <aist_aruco_msgs/msg/point_correspondence_array_array.hpp>
#include <aruco/aruco.h>

namespace aist_aruco_ros
{
/************************************************************************
*  class MultiDetector							*
************************************************************************/
class MultiDetector : public rclcpp::Node
{
  private:
    using image_t	= sensor_msgs::msg::Image;
    using image_cp	= image_t::ConstSharedPtr;
    using correses_set_t= aist_aruco_msgs::msg::PointCorrespondenceArrayArray;
    using mdetector_t	= aruco::MarkerDetector;
    using mparams_t	= mdetector_t::Params;
    using marker_info_t	= aruco::Marker3DInfo;
    using marker_map_t	= aruco::MarkerMap;
    using point2_t	= cv::Point2f;
    using point3_t	= cv::Point3f;

    using ddynamic_reconfigure_t = ddynamic_reconfigure2::DDynamicReconfigure;

    struct rgb_t		{ uint8_t r, g, b; };

    template <size_t N, class... TYPES>
    struct Policy
    {
	using type = typename Policy<N-1, image_t, TYPES...>::type;
    };
    template <class... TYPES>
    struct Policy<0, TYPES...>
    {
	using type = message_filters::sync_policies::ApproximateTime<TYPES...>;
    };

    template <size_t N>
    using policy_t	= typename Policy<N>::type;

    class SyncBase
    {
      public:
	virtual	~SyncBase() = 0;
    };

    template <size_t N>
    class sync_t : public SyncBase
    {
      public:
	template <class CB, class... FILTERS>
		sync_t(const CB& callback, MultiDetector* detector,
		       FILTERS&&... filters)
		    :_sync(policy_t<N>(10), filters...)
		{
		    _sync.registerCallback(callback, detector);
		}
	virtual	~sync_t()						{}

      private:
	message_filters::Synchronizer<policy_t<N> >	_sync;
    };
    using subscriber_t	= image_transport::SubscriberFilter;
    using publisher_t	= image_transport::Publisher;

  public:
		MultiDetector(const rclcpp::NodeOptions& options)	;

  private:
    std::string	node_name()			const	{ return get_name(); }
    void	set_min_marker_size(double size)			;
    void	set_enclosed_marker(bool enable)			;
    void	set_detection_mode(int mode)				;
    void	set_dictionary(const std::string& dict)			;
    template <size_t N, class... IMAGE_MSGS>
    void	image_cb(const image_cp& image_msg,
			 const IMAGE_MSGS&... image_msgs)		;
    template <size_t N>
    void	image_cb()						;
    aist_aruco_msgs::msg::PointCorrespondenceArray
    detect_marker(const image_cp& image_msg,
		  const image_transport::Publisher& result_pub,
		  const image_transport::Publisher& debug_pub,
		  const std::string& camera_name)			;
    static void	publish_image(const std_msgs::msg::Header& header,
			      const cv::Mat& image,
			      const image_transport::Publisher& pub)	;
    const std::string&
		getName()					const	;

  private:
    ddynamic_reconfigure_t				_ddr;

    const std::vector<std::string>			_camera_names;

    image_transport::ImageTransport			_it;
    image_transport::Subscriber				_image_sub;
    std::vector<std::unique_ptr<subscriber_t> >		_image_subs;
    std::vector<publisher_t>				_result_pubs;
    std::vector<publisher_t>				_debug_pubs;
    const rclcpp::Publisher<correses_set_t>::SharedPtr	_corres_pub;
    std::unique_ptr<SyncBase>				_sync;

    tf2_ros::Buffer					_tf2_buffer;
    const tf2_ros::TransformListener			_tf2_listener;
    const std::string					_reference_frame;
    const std::string					_marker_frame;

    mdetector_t						_marker_detector;
    marker_map_t					_marker_map;
    correses_set_t					_correspondences_set;
};

MultiDetector::MultiDetector(const rclcpp::NodeOptions& options)
    :rclcpp::Node("multi_detector", options),
     _ddr(rclcpp::Node::SharedPtr(this)),
     _camera_names(_ddr.declare_read_only_parameter<std::vector<std::string> >(
		       "camera_names", {})),
     _it(rclcpp::Node::SharedPtr(this)),
     _image_sub(),
     _image_subs(),
     _result_pubs(),
     _debug_pubs(),
     _corres_pub(create_publisher<correses_set_t>("point_correspondences_set",
						  1)),
     _sync(),
     _tf2_buffer(get_clock()),
     _tf2_listener(_tf2_buffer),
     _reference_frame(_ddr.declare_read_only_parameter<std::string>(
			  "reference_frame", "")),
     _marker_frame(_ddr.declare_read_only_parameter<std::string>(
		       "marker_frame", "marker_frame")),
     _marker_detector(),
     _marker_map(),
     _correspondences_set()
{
    using namespace	std::placeholders;

    if (_camera_names.empty())
    {
	RCLCPP_ERROR_STREAM(get_logger(), "No camera names specified!");
	return;
    }

    for (const auto& camera_name : _camera_names)
    {
	if (_camera_names.size() >= 2)
	    _image_subs.emplace_back(std::make_unique<subscriber_t>(
					 this,
					 node_name() + '/' + camera_name
						     + "/image", "raw"));
	_result_pubs.emplace_back(_it.advertise(node_name() + '/'
						+ camera_name + "/result",1));
	_debug_pubs .emplace_back(_it.advertise(node_name() + camera_name
						+ "/debug", 1));
	RCLCPP_INFO_STREAM(get_logger(),
			   "Subscribe camera[" << camera_name << ']');
    }

    switch (_camera_names.size())
    {
      case 1:
	_image_sub = _it.subscribe(node_name() + '/' + _camera_names[0]
					       + "/image", 1,
				   &MultiDetector::image_cb<0>, this);
	break;
      case 2:
	_sync.reset(new sync_t<2>(
			&MultiDetector::image_cb<0, image_cp>,
			this,
			*_image_subs[0], *_image_subs[1]));
	break;
      case 3:
	_sync.reset(new sync_t<3>(
			&MultiDetector::image_cb<0, image_cp, image_cp>,
			this,
			*_image_subs[0], *_image_subs[1], *_image_subs[2]));
	break;
      case 4:
	_sync.reset(new sync_t<4>(
			&MultiDetector::image_cb<0, image_cp, image_cp,
						    image_cp>,
			this,
			*_image_subs[0], *_image_subs[1], *_image_subs[2],
			*_image_subs[3]));
	break;
      case 5:
	_sync.reset(new sync_t<5>(
			&MultiDetector::image_cb<0, image_cp, image_cp,
						    image_cp, image_cp>,
			this,
			*_image_subs[0], *_image_subs[1], *_image_subs[2],
			*_image_subs[3], *_image_subs[4]));
	break;
      case 6:
	_sync.reset(new sync_t<6>(
			&MultiDetector::image_cb<0, image_cp, image_cp,
						    image_cp, image_cp,
						    image_cp>,
			this,
			*_image_subs[0], *_image_subs[1], *_image_subs[2],
			*_image_subs[3], *_image_subs[4], *_image_subs[5]));
	break;
      case 7:
	_sync.reset(new sync_t<7>(
			&MultiDetector::image_cb<0, image_cp, image_cp,
						    image_cp, image_cp,
						    image_cp, image_cp>,
			this,
			*_image_subs[0], *_image_subs[1], *_image_subs[2],
			*_image_subs[3], *_image_subs[4], *_image_subs[5],
			*_image_subs[6]));
	break;
      case 8:
	_sync.reset(new sync_t<8>(
			&MultiDetector::image_cb<0, image_cp, image_cp,
						    image_cp, image_cp,
						    image_cp, image_cp,
						    image_cp>,
			this,
			*_image_subs[0], *_image_subs[1], *_image_subs[2],
			*_image_subs[3], *_image_subs[4], *_image_subs[5],
			*_image_subs[6], *_image_subs[7]));
	break;

      default:
	RCLCPP_ERROR_STREAM(get_logger(), "Specified "
			    << _camera_names.size()
			    << " cameras but supported only up to eight!");
	return;
    }

    _correspondences_set.correspondences_set.resize(_camera_names.size());

  // Load marker map.
    const auto marker_map_name = _ddr.declare_read_only_parameter<std::string>(
				     "marker_map", "");
    if (marker_map_name == "")
    {
	RCLCPP_ERROR_STREAM(get_logger(), "Marker map not specified!");
	throw;
    }

    const auto mMapFile = _ddr.declare_read_only_parameter(
			      "marker_map_dir",
			      ament_index_cpp::get_package_share_directory(
				  "aist_aruco_ros")
			      + "/config")
			+ '/' + marker_map_name + ".yaml";
    try
    {
	_marker_map.readFromFile(mMapFile);

	RCLCPP_INFO_STREAM(get_logger(),
			   "Loaded marker map[" << mMapFile << ']');
    }
    catch (const std::exception& err)
    {
	RCLCPP_ERROR_STREAM(get_logger(),
			    "Failed to read marker map[" << mMapFile << ']');
	throw;
    }

  // Set minimum marker size and setup ddynamic_reconfigure service for it.
    _ddr.registerVariable<double>("min_marker_size",
				  _marker_detector.getParameters().minSize,
				  std::bind(
				      &MultiDetector::set_min_marker_size,
				      this, _1),
				  "Minimum marker size", {0.0, 1.0});

  // Set a parameter for specifying whether the marker is enclosed or not.
    _ddr.registerVariable<bool>("enclosed_marker",
				_marker_detector.getParameters().enclosedMarker,
				std::bind(
				    &MultiDetector::set_enclosed_marker,
				    this, _1),
				"Detect enclosed marker");

  // Set detection mode and setup ddynamic_reconfigure service for it.
    _ddr.registerEnumVariable<int>("detection_mode",
				   _marker_detector.getDetectionMode(),
				   std::bind(
				       &MultiDetector::set_detection_mode,
				       this, _1),
				   "Marker detection mode",
				   {{"NORMAL",	   aruco::DM_NORMAL},
				    {"FAST",	   aruco::DM_FAST},
				    {"VIDEO_FAST", aruco::DM_VIDEO_FAST}});

  // Set dictionary and setup ddynamic_reconfigure service for it.
    const std::string	dict = "ARUCO";
    set_dictionary(dict);
    _ddr.registerEnumVariable<std::string>(
	"dictionary", dict,
	std::bind(&MultiDetector::set_dictionary, this, _1),
	"Dictionary",
	{{"ARUCO",		"ARUCO"},
	 {"ARUCO_MIP_25h7",	"ARUCO_MIP_25h7"},
	 {"ARUCO_MIP_16h3",	"ARUCO_MIP_16h3"},
	 {"ARTAG",		"ARTAG"},
	 {"ARTOOLKITPLUS",	"ARTOOLKITPLUS"},
	 {"ARTOOLKITPLUSBCH",	"ARTOOLKITPLUSBCH"},
	 {"TAG16h5",		"TAG16h5"},
	 {"TAG25h7",		"TAG25h7"},
	 {"TAG25h9",		"TAG25h9"},
	 {"TAG36h11",		"TAG36h11"},
	 {"TAG36h10",		"TAG36h10"},
	 {"CUSTOM",		"CUSTOM"}});
}

void
MultiDetector::set_min_marker_size(double size)
{
    _marker_detector.setDetectionMode(_marker_detector.getDetectionMode(),
				      size);

    RCLCPP_INFO_STREAM(get_logger(), "Set min_marker_size[" << size << ']');
}

void
MultiDetector::set_enclosed_marker(bool enable)
{
    _marker_detector.getParameters().detectEnclosedMarkers(enable);

    RCLCPP_INFO_STREAM(get_logger(), "Set enclosed_marker["
		       << std::boolalpha << enable << ']');
}

void
MultiDetector::set_detection_mode(int mode)
{
    _marker_detector.setDetectionMode(aruco::DetectionMode(mode),
				      _marker_detector.getParameters().minSize);

    RCLCPP_INFO_STREAM(get_logger(), "Set detection_mode[" << mode << ']');
}

void
MultiDetector::set_dictionary(const std::string& dict)
{
    _marker_detector.setDictionary(dict);
    _marker_map.setDictionary(dict);

    RCLCPP_INFO_STREAM(get_logger(), "Set dictionary[" << dict << ']');
}

template <size_t N, class... IMAGE_MSGS> void
MultiDetector::image_cb(const image_cp& image_msg,
			const IMAGE_MSGS&... image_msgs)
{
    try
    {
	_correspondences_set.correspondences_set[N]
	    = detect_marker(image_msg, _result_pubs[N], _debug_pubs[N],
			    _camera_names[N]);
    }
    catch (const std::runtime_error& err)
    {
	RCLCPP_ERROR_STREAM(get_logger(), "Failed to detect marker for "
			     << N << "-th camera: " << err.what());
	return;
    }

    image_cb<N+1>(image_msgs...);
}

template <size_t N> void
MultiDetector::image_cb()
{
    _corres_pub->publish(_correspondences_set);
}

aist_aruco_msgs::msg::PointCorrespondenceArray
MultiDetector::detect_marker(const image_cp& image_msg,
			     const image_transport::Publisher& result_pub,
			     const image_transport::Publisher& debug_pub,
			     const std::string& camera_name)
{
    using namespace	sensor_msgs;

  // Convert the input image message to OpenCV's matrix type.
    auto image = cv_bridge::toCvCopy(image_msg, image_encodings::RGB8)->image;

  // Detect markers. Results will go into "markers".
    const auto	markers = _marker_detector.detect(image);

  // Show also the internal image resulting from the threshold operation.
    publish_image(image_msg->header, _marker_detector.getThresholdedImage(),
     		  debug_pub);

  // Create correpondence pairs of source and image points.
    std::vector<std::pair<point3_t, point2_t> >	pairs;
    for (const auto& marker : markers)
    {
      // For each marker, draw info and its boundaries in the image.
	marker.draw(image, cv::Scalar(0, 0, 255), 2);

	const auto	i = _marker_map.getIndexOfMarkerId(marker.id);
	if (i < 0 || marker.size() < 4)
	    continue;

      // Append pairs of the source marker corners and corresponding
      // image points detected.
	const auto&	markerinfo = _marker_map[i];
	for (size_t j = 0; j < marker.size(); ++j)
	    pairs.emplace_back(markerinfo[j], marker[j]);
    }

  // Publish the input image with detected marker drawen.
    publish_image(image_msg->header, image, result_pub);

  // Create point correspondences for this image.
    aist_aruco_msgs::msg::PointCorrespondenceArray	correspondences;
    correspondences.header	    = image_msg->header;
    correspondences.height	    = image_msg->height;
    correspondences.width	    = image_msg->width;
    correspondences.camera_name	    = camera_name;
    correspondences.reference_frame = _reference_frame;
    for (const auto& pair : pairs)
    {
	aist_aruco_msgs::msg::PointCorrespondence	correspondence;
	correspondence.source_point.x = pair.first.x;
	correspondence.source_point.y = pair.first.y;
	correspondence.source_point.z = pair.first.z;
	correspondence.image_point.x  = pair.second.x;
	correspondence.image_point.y  = pair.second.y;
	correspondence.image_point.z  = 0;

	correspondences.correspondences.push_back(correspondence);
    }

  // Transform source points from marker frame to reference frame
  // if the reference frame is explicitly specified.
    if (!_reference_frame.empty())
    {
	const auto Trm = _tf2_buffer.lookupTransform(_reference_frame,
						     _marker_frame,
						     image_msg->header.stamp,
						     tf2::durationFromSec(1.0));
	for (auto&& correspondence : correspondences.correspondences)
	    tf2::doTransform(correspondence.source_point,
			     correspondence.source_point, Trm);
    }

    return correspondences;
}

void
MultiDetector::publish_image(const std_msgs::msg::Header& header,
			     const cv::Mat& image,
			     const image_transport::Publisher& pub)
{
    using namespace	sensor_msgs;

    if (pub.getNumSubscribers() > 0)
	pub.publish(cv_bridge::CvImage(header, image_encodings::RGB8, image)
		    .toImageMsg());
}
}	// namespace aist_aruco_ros

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(aist_aruco_ros::MultiDetector)
