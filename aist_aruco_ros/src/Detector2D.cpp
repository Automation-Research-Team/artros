/*!
* \file		Detector2D.cpp
* \author	Toshio UESHIBA
* \brief	ARuCo marker detector using intensity images
*/
#include <iostream>
#include <cstdint>

#include <ros/package.h>
#include <image_transport/image_transport.h>
#include <ddynamic_reconfigure/ddynamic_reconfigure.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <aist_utility/opencv.h>
#include <aist_utility/sensor_msgs.h>
#include <aist_aruco_ros/PointCorrespondenceArray.h>
#include <aruco/aruco.h>

namespace aist_aruco_ros
{
/************************************************************************
*  class Detector2D							*
************************************************************************/
class Detector2D
{
  private:
    using image_t		= sensor_msgs::Image;
    using image_p		= sensor_msgs::ImageConstPtr;
    using mdetector_t		= aruco::MarkerDetector;
    using mparams_t		= mdetector_t::Params;
    using marker_info_t		= aruco::Marker3DInfo;
    using marker_map_t		= aruco::MarkerMap;
    using point2_t		= cv::Point2f;
    using point3_t		= cv::Point3f;
    using ddynamic_reconfigure_t= ddynamic_reconfigure::DDynamicReconfigure;

    struct rgb_t		{ uint8_t r, g, b; };

  public:
		Detector2D(ros::NodeHandle& nh)				;

  private:
    void	set_min_marker_size(double size)			;
    void	set_enclosed_marker(bool enable)			;
    void	set_detection_mode(int mode)				;
    void	set_dictionary(const std::string& dict)			;
    void	image_cb(const image_p&	 image_msg)			;
    static void	publish_image(const std_msgs::Header& header,
			      const cv::Mat& image,
			      const image_transport::Publisher& pub)	;
    template <class ITER>
    void	publish_correspondences(ITER begin, ITER end,
					const std_msgs::Header& header)	;

  private:
    image_transport::ImageTransport	_it;
    image_transport::Subscriber		_image_sub;
    const image_transport::Publisher	_result_pub;
    const image_transport::Publisher	_debug_pub;
    const ros::Publisher		_corres_pub;

    ddynamic_reconfigure_t		_ddr;

    mdetector_t				_marker_detector;
    marker_map_t			_marker_map;
};

Detector2D::Detector2D(ros::NodeHandle& nh)
    :_it(nh),
     _image_sub(_it.subscribe("/image", 1, &Detector2D::image_cb, this)),
     _result_pub(_it.advertise("result", 1)),
     _debug_pub( _it.advertise("debug",  1)),
     _corres_pub(nh.advertise<PointCorrespondenceArray>(
		     "point_correspondences", 100)),
     _ddr(nh),
     _marker_detector(),
     _marker_map()
{
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

	ROS_INFO_STREAM("Find a marker map[" << marker_map_name << ']');
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

	ROS_INFO_STREAM("Find a marker with ID[" << marker_id << ']');
    }
    else
    {
	throw std::runtime_error("Neither marker map nor marker ID specified!");
    }

  // Set minimum marker size and setup ddynamic_reconfigure service for it.
    _ddr.registerVariable<double>("min_marker_size",
				  _marker_detector.getParameters().minSize,
				  boost::bind(&Detector2D::set_min_marker_size,
					      this, _1),
				  "Minimum marker size", 0.0, 1.0);
    ROS_INFO_STREAM("Minimum marker size: "
		    << _marker_detector.getParameters().minSize);

  // Set a parameter for specifying whether the marker is enclosed or not.
    _ddr.registerVariable<bool>("enclosed_marker",
				_marker_detector.getParameters().enclosedMarker,
				boost::bind(&Detector2D::set_enclosed_marker,
					    this, _1),
				"Detect enclosed marker", false, true);
    ROS_INFO_STREAM("Detect enclosed marker: "
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
				   boost::bind(&Detector2D::set_detection_mode,
					       this, _1),
				   "Marker detection mode",
				   map_detectionMode);
    ROS_INFO_STREAM("Detection mode: " << _marker_detector.getDetectionMode());

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
	"dictionary", dict, boost::bind(&Detector2D::set_dictionary, this, _1),
	"Dictionary", map_dict);
    ROS_INFO_STREAM("Dictionary: " << dict);

  // Pulish ddynamic_reconfigure service.
    _ddr.publishServicesTopicsAndUpdateConfigData();
}

void
Detector2D::set_min_marker_size(double size)
{
    _marker_detector.setDetectionMode(_marker_detector.getDetectionMode(),
				      size);
}

void
Detector2D::set_enclosed_marker(bool enable)
{
    _marker_detector.getParameters().detectEnclosedMarkers(enable);
}

void
Detector2D::set_detection_mode(int mode)
{
    _marker_detector.setDetectionMode(aruco::DetectionMode(mode),
				      _marker_detector.getParameters().minSize);
}

void
Detector2D::set_dictionary(const std::string& dict)
{
    _marker_detector.setDictionary(dict);
    _marker_map.setDictionary(dict);
}

void
Detector2D::image_cb(const image_p& image_msg)
{
    using namespace	sensor_msgs;

    auto	image = cv_bridge::toCvCopy(image_msg, image_encodings::RGB8)
		      ->image;

  // Detect markers. Results will go into "markers"
    const auto	markers = _marker_detector.detect(image);

  // Show also the internal image resulting from the threshold operation
    publish_image(image_msg->header, _marker_detector.getThresholdedImage(),
		  _debug_pub);

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

    publish_image(image_msg->header, image, _result_pub);

    publish_correspondences(pairs.begin(), pairs.end(), image_msg->header);
}

void
Detector2D::publish_image(const std_msgs::Header& header, const cv::Mat& image,
			  const image_transport::Publisher& pub)
{
    using namespace	sensor_msgs;

    if (pub.getNumSubscribers() > 0)
	pub.publish(cv_bridge::CvImage(header, image_encodings::RGB8, image)
		    .toImageMsg());
}

template <class ITER> void
Detector2D::publish_correspondences(ITER begin, ITER end,
				    const std_msgs::Header& header)
{
    PointCorrespondenceArray	correses;
    correses.header = header;

    for (auto iter = begin; iter != end; ++iter)
    {
	PointCorrespondence	corres;
	corres.point.x	     = iter->first.x;
	corres.point.y	     = iter->first.y;
	corres.point.z	     = iter->first.z;
	corres.image_point.u = iter->second.x;
	corres.image_point.v = iter->second.y;

	correses.point_correspondences.push_back(corres);
    }

    _corres_pub.publish(correses);
}

/************************************************************************
*  class Detector2DNodelet						*
************************************************************************/
class Detector2DNodelet : public nodelet::Nodelet
{
  public:
			Detector2DNodelet()				{}

    virtual void	onInit()					;

  private:
    boost::shared_ptr<Detector2D>	_node;
};

void
Detector2DNodelet::onInit()
{
    NODELET_INFO("aist_aruco_ros::Detector2DNodelet::onInit()");
    _node.reset(new Detector2D(getPrivateNodeHandle()));
}

}	// namespace aist_aruco_ros

PLUGINLIB_EXPORT_CLASS(aist_aruco_ros::Detector2DNodelet, nodelet::Nodelet);
