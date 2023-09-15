/*!
* \file		MultiDetector2D.cpp
* \author	Toshio UESHIBA
* \brief	ARuCo marker detector using intensity images
*/
#include <iostream>
#include <cstdint>
#include <any>

#include <ros/package.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <ddynamic_reconfigure/ddynamic_reconfigure.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <aist_utility/opencv.h>
#include <aist_utility/sensor_msgs.h>
#include <aist_aruco_ros/PointCorrespondenceArrayArray.h>
#include <aruco/aruco.h>

namespace aist_aruco_ros
{
/************************************************************************
*  class MultiDetector2D						*
************************************************************************/
class MultiDetector2D
{
  private:
    using image_t	= sensor_msgs::Image;
    using image_cp	= sensor_msgs::ImageConstPtr;
    using mdetector_t	= aruco::MarkerDetector;
    using mparams_t	= mdetector_t::Params;
    using marker_info_t	= aruco::Marker3DInfo;
    using marker_map_t	= aruco::MarkerMap;
    using point2_t	= cv::Point2f;
    using point3_t	= cv::Point3f;

    using ddynamic_reconfigure_t = ddynamic_reconfigure::DDynamicReconfigure;

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
		sync_t(const CB& callback, MultiDetector2D* detector,
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
		MultiDetector2D(ros::NodeHandle& nh,
				const std::string& nodelet_name)	;

  private:
    void	set_min_marker_size(double size)			;
    void	set_enclosed_marker(bool enable)			;
    void	set_detection_mode(int mode)				;
    void	set_dictionary(const std::string& dict)			;
    template <size_t N, class... IMAGE_MSGS>
    void	image_cb(const image_cp& image_msg,
			 const IMAGE_MSGS&... image_msgs)		;
    template <size_t N>
    void	image_cb()						;
    aist_aruco_ros::PointCorrespondenceArray
    detect_marker(const image_cp& image_msg,
		  const image_transport::Publisher& result_pub,
		  const image_transport::Publisher& debug_pub)		;
    static void	publish_image(const std_msgs::Header& header,
			      const cv::Mat& image,
			      const image_transport::Publisher& pub)	;
    template <class ITER>
    void	publish_correspondences(ITER begin, ITER end,
					const std_msgs::Header& header)	;
    const std::string&
		getName()					const	;

  private:
    const std::string					_nodelet_name;
    
    image_transport::ImageTransport			_it;
    image_transport::Subscriber				_image_sub;
    std::vector<std::unique_ptr<subscriber_t> >		_image_subs;
    std::vector<publisher_t>				_result_pubs;
    std::vector<publisher_t>				_debug_pubs;
    const ros::Publisher				_corres_pub;
    std::unique_ptr<SyncBase>				_sync;

    ddynamic_reconfigure_t				_ddr;

    mdetector_t						_marker_detector;
    marker_map_t					_marker_map;
    aist_aruco_ros::PointCorrespondenceArrayArray	_correspondences_set;
};

MultiDetector2D::MultiDetector2D(ros::NodeHandle& nh,
				 const std::string& nodelet_name)
    :_nodelet_name(nodelet_name),
     _it(nh),
     _image_sub(),
     _image_subs(),
     _result_pubs(),
     _debug_pubs(),
     _corres_pub(nh.advertise<PointCorrespondenceArrayArray>(
		     "point_correspondences_set", 100)),
     _sync(),
     _ddr(nh),
     _marker_detector(),
     _marker_map(),
     _correspondences_set()
{
    std::vector<std::string>	camera_names;
    if (!nh.getParam("camera_names", camera_names) || camera_names.size() == 0)
    {
	NODELET_ERROR_STREAM("(MultiDetector2D) No camera names specified!");
	return;
    }

    for (const auto& camera_name : camera_names)
    {
	if (camera_names.size() >= 2)
	    _image_subs.emplace_back(std::make_unique<subscriber_t>(
					 _it, camera_name + "/image", 1));
	_result_pubs.emplace_back(_it.advertise(camera_name + "/result",1));
	_debug_pubs .emplace_back(_it.advertise(camera_name + "/debug", 1));
	NODELET_INFO_STREAM("(MultiDetector2D) Subscribe camera["
			    << camera_name << ']');
    }

    switch (camera_names.size())
    {
      case 1:
	_image_sub = _it.subscribe(camera_names[0] + "/image", 1,
				   &MultiDetector2D::image_cb<0>, this);
	break;
      case 2:
	_sync.reset(new sync_t<2>(
			&MultiDetector2D::image_cb<0, image_cp>,
			this,
			*_image_subs[0], *_image_subs[1]));
	break;
      case 3:
	_sync.reset(new sync_t<3>(
			&MultiDetector2D::image_cb<0, image_cp, image_cp>,
			this,
			*_image_subs[0], *_image_subs[1], *_image_subs[2]));
	break;
      case 4:
	_sync.reset(new sync_t<4>(
			&MultiDetector2D::image_cb<0, image_cp, image_cp,
						   image_cp>,
			this,
			*_image_subs[0], *_image_subs[1], *_image_subs[2],
			*_image_subs[3]));
	break;
      case 5:
	_sync.reset(new sync_t<5>(
			&MultiDetector2D::image_cb<0, image_cp, image_cp,
						   image_cp, image_cp>,
			this,
			*_image_subs[0], *_image_subs[1], *_image_subs[2],
			*_image_subs[3], *_image_subs[4]));
	break;
      case 6:
	_sync.reset(new sync_t<6>(
			&MultiDetector2D::image_cb<0, image_cp, image_cp,
						   image_cp, image_cp,
						   image_cp>,
			this,
			*_image_subs[0], *_image_subs[1], *_image_subs[2],
			*_image_subs[3], *_image_subs[4], *_image_subs[5]));
	break;
      case 7:
	_sync.reset(new sync_t<7>(
			&MultiDetector2D::image_cb<0, image_cp, image_cp,
						   image_cp, image_cp,
						   image_cp, image_cp>,
			this,
			*_image_subs[0], *_image_subs[1], *_image_subs[2],
			*_image_subs[3], *_image_subs[4], *_image_subs[5],
			*_image_subs[6]));
	break;
      case 8:
	_sync.reset(new sync_t<8>(
			&MultiDetector2D::image_cb<0, image_cp, image_cp,
						   image_cp, image_cp,
						   image_cp, image_cp,
						   image_cp>,
			this,
			*_image_subs[0], *_image_subs[1], *_image_subs[2],
			*_image_subs[3], *_image_subs[4], *_image_subs[5],
			*_image_subs[6], *_image_subs[7]));
	break;

      default:
	NODELET_ERROR_STREAM("(MultiDetector2D) Specified "
			     << camera_names.size()
			     << " cameras but supported only up to eight!");
	return;
    }

    _correspondences_set.correspondences_set.resize(camera_names.size());
    
  // Restore marker map if specified.
    if (const auto marker_map_name = nh.param<std::string>("marker_map", "");
	marker_map_name != "")
    {
	const auto	mMapFile = nh.param("marker_map_dir",
					    ros::package::getPath(
						"aist_aruco_ros")
					    + "/config")
				 + '/' + marker_map_name + ".yaml";
	_marker_map.readFromFile(mMapFile);

	NODELET_INFO_STREAM("(MultiDetector2D) Let's find marker map["
			    << marker_map_name << ']');
    }
    else if (const auto marker_id = nh.param<int>("marker_id", -1);
	     marker_id >= 0)
    {
	const auto	half_size = nh.param("marker_size", 0.05f) / 2;
	marker_info_t	mInfo(marker_id);
	mInfo.push_back({-half_size,  half_size, 0});
	mInfo.push_back({ half_size,  half_size, 0});
	mInfo.push_back({ half_size, -half_size, 0});
	mInfo.push_back({-half_size, -half_size, 0});
	_marker_map.push_back(mInfo);
	_marker_map.mInfoType = marker_map_t::METERS;

	NODELET_INFO_STREAM("(MultiDetector2D) Let's find a marker with ID["
			    << marker_id << ']');
    }
    else
    {
	throw std::runtime_error("Neither marker map nor marker ID specified!");
    }

  // Set minimum marker size and setup ddynamic_reconfigure service for it.
    _ddr.registerVariable<double>("min_marker_size",
				  _marker_detector.getParameters().minSize,
				  boost::bind(
				      &MultiDetector2D::set_min_marker_size,
				      this, _1),
				  "Minimum marker size", 0.0, 1.0);
    NODELET_INFO_STREAM("(MultiDetector2D) Minimum marker size: "
			<< _marker_detector.getParameters().minSize);

  // Set a parameter for specifying whether the marker is enclosed or not.
    _ddr.registerVariable<bool>("enclosed_marker",
				_marker_detector.getParameters().enclosedMarker,
				boost::bind(
				    &MultiDetector2D::set_enclosed_marker,
				    this, _1),
				"Detect enclosed marker", false, true);
    NODELET_INFO_STREAM("(MultiDetector2D) Detect enclosed marker: "
			<< std::boolalpha
			<< _marker_detector.getParameters().enclosedMarker);

  // Set detection mode and setup ddynamic_reconfigure service for it.
    std::map<std::string, int>	map_detectionMode =
    				{
				    {"NORMAL",	   aruco::DM_NORMAL},
				    {"FAST",	   aruco::DM_FAST},
				    {"VIDEO_FAST", aruco::DM_VIDEO_FAST},
				};
    _ddr.registerEnumVariable<int>("detection_mode",
				   _marker_detector.getDetectionMode(),
				   boost::bind(
				       &MultiDetector2D::set_detection_mode,
				       this, _1),
				   "Marker detection mode",
				   map_detectionMode);
    NODELET_INFO_STREAM("(MultiDetector2D) Detection mode: "
			<< _marker_detector.getDetectionMode());

  // Set dictionary and setup ddynamic_reconfigure service for it.
    const auto	dict = nh.param<std::string>("dictionary", "ARUCO");
    set_dictionary(dict);

    std::map<std::string, std::string>
	map_dict =
	{
	    {"ARUCO",		 "ARUCO"},
	    {"ARUCO_MIP_25h7",	 "ARUCO_MIP_25h7"},
	    {"ARUCO_MIP_16h3",	 "ARUCO_MIP_16h3"},
	    {"ARTAG",		 "ARTAG"},
	    {"ARTOOLKITPLUS",	 "ARTOOLKITPLUS"},
	    {"ARTOOLKITPLUSBCH", "ARTOOLKITPLUSBCH"},
	    {"TAG16h5",		 "TAG16h5"},
	    {"TAG25h7",		 "TAG25h7"},
	    {"TAG25h9",		 "TAG25h9"},
	    {"TAG36h11",	 "TAG36h11"},
	    {"TAG36h10",	 "TAG36h10"},
	    {"CUSTOM",		 "CUSTOM"},
	};
    _ddr.registerEnumVariable<std::string>(
	"dictionary", dict,
	boost::bind(&MultiDetector2D::set_dictionary, this, _1),
	"Dictionary", map_dict);
    NODELET_INFO_STREAM("(MultiDetector2D) Dictionary: " << dict);

  // Pulish ddynamic_reconfigure service.
    _ddr.publishServicesTopicsAndUpdateConfigData();
}

void
MultiDetector2D::set_min_marker_size(double size)
{
    _marker_detector.setDetectionMode(_marker_detector.getDetectionMode(),
				      size);
}

void
MultiDetector2D::set_enclosed_marker(bool enable)
{
    _marker_detector.getParameters().detectEnclosedMarkers(enable);
}

void
MultiDetector2D::set_detection_mode(int mode)
{
    _marker_detector.setDetectionMode(aruco::DetectionMode(mode),
				      _marker_detector.getParameters().minSize);
}

void
MultiDetector2D::set_dictionary(const std::string& dict)
{
    _marker_detector.setDictionary(dict);
    _marker_map.setDictionary(dict);
}

template <size_t N, class... IMAGE_MSGS> void
MultiDetector2D::image_cb(const image_cp& image_msg,
			  const IMAGE_MSGS&... image_msgs)
{
    _correspondences_set.correspondences_set[N]
	= detect_marker(image_msg, _result_pubs[N], _debug_pubs[N]);
    image_cb<N+1>(image_msgs...);
}

template <size_t N> void
MultiDetector2D::image_cb()
{
    _corres_pub.publish(_correspondences_set);
}

aist_aruco_ros::PointCorrespondenceArray
MultiDetector2D::detect_marker(const image_cp& image_msg,
			       const image_transport::Publisher& result_pub,
			       const image_transport::Publisher& debug_pub)
{
    using namespace	sensor_msgs;

  // Convert the input image message to OpenCV's matrix type.
    auto image = cv_bridge::toCvCopy(image_msg, image_encodings::RGB8)->image;

  // Detect markers. Results will go into "markers".
    const auto	markers = _marker_detector.detect(image);

  // Show also the internal image resulting from the threshold operation.
    publish_image(image_msg->header, _marker_detector.getThresholdedImage(),
     		  debug_pub);

    std::vector<std::pair<point3_t, point2_t> >	pairs;

    for (const auto& marker : markers)
    {
      // For each marker, draw info and its boundaries in the image.
	marker.draw(image, cv::Scalar(0, 0, 255), 2);

	const auto	i = _marker_map.getIndexOfMarkerId(marker.id);

	if (i < 0 || marker.size() < 4)
	    continue;

	const auto&	markerinfo = _marker_map[i];
	for (size_t j = 0; j < marker.size(); ++j)
	    pairs.push_back(std::make_pair(markerinfo[j], marker[j]));
    }

    publish_image(image_msg->header, image, result_pub);

  // Create point correspondences for this image.
    PointCorrespondenceArray	correspondences;
    correspondences.header = image_msg->header;
    for (const auto& pair : pairs)
    {
	PointCorrespondence	correspondence;
	correspondence.point.x	     = pair.first.x;
	correspondence.point.y	     = pair.first.y;
	correspondence.point.z	     = pair.first.z;
	correspondence.image_point.x = pair.second.x;
	correspondence.image_point.y = pair.second.y;
	correspondence.image_point.z = 0;

	correspondences.correspondences.push_back(correspondence);
    }

    return correspondences;
}
			       
void
MultiDetector2D::publish_image(const std_msgs::Header& header,
			       const cv::Mat& image,
			       const image_transport::Publisher& pub)
{
    using namespace	sensor_msgs;

    if (pub.getNumSubscribers() > 0)
	pub.publish(cv_bridge::CvImage(header, image_encodings::RGB8, image)
		    .toImageMsg());
}

const std::string&
MultiDetector2D::getName() const
{
    return _nodelet_name;
}

/************************************************************************
*  class MultiDetector2DNodelet						*
************************************************************************/
class MultiDetector2DNodelet : public nodelet::Nodelet
{
  public:
			MultiDetector2DNodelet()			{}

    virtual void	onInit()					;

  private:
    boost::shared_ptr<MultiDetector2D>	_node;
};

void
MultiDetector2DNodelet::onInit()
{
    NODELET_INFO("aist_aruco_ros::MultiDetector2DNodelet::onInit()");
    _node.reset(new MultiDetector2D(getPrivateNodeHandle(), getName()));
}

}	// namespace aist_aruco_ros

PLUGINLIB_EXPORT_CLASS(aist_aruco_ros::MultiDetector2DNodelet,
		       nodelet::Nodelet);
