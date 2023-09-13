/*!
* \file		Detector.cpp
* \author	Toshio UESHIBA
* \brief	ARuCo marker detector using both intensity and depth images
*/
#include <iostream>
#include <limits>
#include <cstdint>

#include <ros/package.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>
#include <visualization_msgs/Marker.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <ddynamic_reconfigure/ddynamic_reconfigure.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <opencv2/calib3d.hpp>
#include <cv_bridge/cv_bridge.h>
#include <aruco/aruco.h>
#include <aist_utility/opencv.h>
#include <aist_utility/sensor_msgs.h>

namespace aist_aruco_ros
{
/************************************************************************
*  static functions							*
************************************************************************/
template <class T> inline T
val(const sensor_msgs::Image& image_msg, int u, int v)
{
    using namespace	sensor_msgs;

    if (image_msg.encoding == image_encodings::TYPE_16UC1)
	return T(0.001) * *reinterpret_cast<const uint16_t*>(
				image_msg.data.data() + v*image_msg.step
						      + u*sizeof(uint16_t));
    else
	return *reinterpret_cast<const T*>(image_msg.data.data()
					   + v*image_msg.step + u*sizeof(T));
}

static aruco::CameraParameters
rosCameraInfo2ArucoCamParams(const sensor_msgs::CameraInfo& camera_info,
			     bool useRectifiedParameters)
{
    cv::Mat		cameraMatrix(3, 3, CV_64FC1, 0.0);
    cv::Mat		distorsionCoeff(4, 1, CV_64FC1, 0.0);
    const cv::Size	size(camera_info.width, camera_info.height);

    if (useRectifiedParameters)
    {
	cameraMatrix.at<double>(0, 0) = camera_info.P[0];
	cameraMatrix.at<double>(0, 1) = camera_info.P[1];
	cameraMatrix.at<double>(0, 2) = camera_info.P[2];
	cameraMatrix.at<double>(1, 0) = camera_info.P[4];
	cameraMatrix.at<double>(1, 1) = camera_info.P[5];
	cameraMatrix.at<double>(1, 2) = camera_info.P[6];
	cameraMatrix.at<double>(2, 0) = camera_info.P[8];
	cameraMatrix.at<double>(2, 1) = camera_info.P[9];
	cameraMatrix.at<double>(2, 2) = camera_info.P[10];
    }
    else
    {
	for (int i = 0; i < 9; ++i)
	    *(cameraMatrix.ptr<double>() + i) = camera_info.K[i];

	for (int i = 0; i < std::min(size_t(4), camera_info.D.size()); ++i)
	    *(distorsionCoeff.ptr<double>() + i) = camera_info.D[i];
    }

    return {cameraMatrix, distorsionCoeff, size};
}

/************************************************************************
*  class Detector							*
************************************************************************/
class Detector
{
  private:
    using camera_info_t		= sensor_msgs::CameraInfo;
    using camera_info_cp	= sensor_msgs::CameraInfoConstPtr;
    using image_t		= sensor_msgs::Image;
    using image_cp		= sensor_msgs::ImageConstPtr;
    using cloud_t		= sensor_msgs::PointCloud2;
    using cloud_cp		= sensor_msgs::PointCloud2ConstPtr;
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
    void	set_min_marker_size(double size)			;
    void	set_enclosed_marker(bool enable)			;
    void	set_detection_mode(int mode)				;
    void	set_dictionary(const std::string& dict)			;
    void	detect_marker_from_depth_cb(
		    const image_cp&	  image_msg,
		    const image_cp&	  depth_msg,
		    const camera_info_cp& camera_info_msg)		;
    void	detect_marker_from_cloud_cb(
		    const cloud_cp&	  cloud_msg,
		    const camera_info_cp& camera_info_msg)		;
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

Detector::Detector(ros::NodeHandle& nh)
    :_broadcaster(),
     _marker_frame(nh.param<std::string>("marker_frame", "marker_frame")),
     _it(nh),
     _image_sub(_it, "/image", 1),
     _depth_sub(_it, "/depth", 1),
     _cloud_sub( nh, "/pointcloud", 1),
     _camera_info_sub(nh, "/camera_info", 1),
     _depth_sync(_image_sub, _depth_sub, _camera_info_sub, 3),
     _cloud_sync(_cloud_sub, _camera_info_sub, 3),
     _camParam(),
     _useRectifiedImages(nh.param("image_is_rectified", false)),
     _rightToLeft(),
     _result_pub(_it.advertise("result", 1)),
     _debug_pub( _it.advertise("debug",  1)),
     _pose_pub(nh.advertise<geometry_msgs::PoseStamped>("pose", 100)),
     _ddr(nh),
     _marker_detector(),
     _marker_size(nh.param("marker_size", 0.05)),
     _useSimilarity(false),
     _planarityTolerance(0.001)
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
	_marker_size = cv::norm(_marker_map[0].points[0] -
				_marker_map[0].points[1]);

	ROS_INFO_STREAM("Find a marker map[" << marker_map_name << ']');
    }
    else if (const auto marker_id = nh.param<int>("marker_id", 0);
	     marker_id > 0)
    {
	const auto	half_size = _marker_size/2;
	marker_info_t	mInfo(marker_id);
	mInfo.push_back({-half_size,  half_size, 0});
	mInfo.push_back({ half_size,  half_size, 0});
	mInfo.push_back({ half_size, -half_size, 0});
	mInfo.push_back({-half_size, -half_size, 0});
	_marker_map.push_back(mInfo);
	_marker_map.mInfoType = marker_map_t::METERS;

	ROS_INFO_STREAM("Find a marker with ID[" << marker_id << ']');
    }

  // Set minimum marker size and setup ddynamic_reconfigure service for it.
    _ddr.registerVariable<double>("min_marker_size",
				  _marker_detector.getParameters().minSize,
				  boost::bind(&Detector::set_min_marker_size,
					      this, _1),
				  "Minimum marker size", 0.0, 1.0);
    ROS_INFO_STREAM("Minimum marker size: "
		    << _marker_detector.getParameters().minSize);

  // Set a parameter for specifying whether the marker is enclosed or not.
    _ddr.registerVariable<bool>("enclosed_marker",
				_marker_detector.getParameters().enclosedMarker,
				boost::bind(&Detector::set_enclosed_marker,
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
				   boost::bind(&Detector::set_detection_mode,
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
	"dictionary", dict, boost::bind(&Detector::set_dictionary, this, _1),
	"Dictionary", map_dict);
    ROS_INFO_STREAM("Dictionary: " << dict);

  // Set usage of rigid transformation and setup its dynamic reconfigure service.
    _ddr.registerVariable<bool>(
	"use_similarity", &_useSimilarity,
	"Use similarity transformation to determine marker poses.",
	false, true);

  // Set planarity tolerance and setup its ddynamic_recoconfigure service.
    _ddr.registerVariable<double>(
    	"planarity_tolerance", &_planarityTolerance,
    	"Planarity tolerance for extracting marker region(in meters)",
    	0.0005, 0.05);

  // Pulish ddynamic_reconfigure service.
    _ddr.publishServicesTopicsAndUpdateConfigData();

  // Register callback for marker detection.
    _depth_sync.registerCallback(&Detector::detect_marker_from_depth_cb, this);
    _cloud_sync.registerCallback(&Detector::detect_marker_from_cloud_cb, this);
}

void
Detector::set_min_marker_size(double size)
{
    _marker_detector.setDetectionMode(_marker_detector.getDetectionMode(),
				      size);
}

void
Detector::set_enclosed_marker(bool enable)
{
    _marker_detector.getParameters().detectEnclosedMarkers(enable);
}

void
Detector::set_detection_mode(int mode)
{
    _marker_detector.setDetectionMode(aruco::DetectionMode(mode),
				      _marker_detector.getParameters().minSize);
}

void
Detector::set_dictionary(const std::string& dict)
{
    _marker_detector.setDictionary(dict);
    _marker_map.setDictionary(dict);
}

void
Detector::detect_marker_from_depth_cb(const image_cp& image_msg,
				      const image_cp& depth_msg,
				      const camera_info_cp& camera_info_msg)
{
    using namespace	sensor_msgs;

    auto	image = cv_bridge::toCvCopy(image_msg, image_encodings::RGB8)
		      ->image;
    detect_marker(*depth_msg, *camera_info_msg, image);
}

void
Detector::detect_marker_from_cloud_cb(const cloud_cp& cloud_msg,
				      const camera_info_cp& camera_info_msg)
{
    if (cloud_msg->is_dense)
    {
	ROS_ERROR_STREAM("Ordered point cloud required! Cloud with size["
			 << cloud_msg->width << 'x' << cloud_msg->height
			 << "] supplied.");
	return;
    }

    cv::Mat	image(cloud_msg->height, cloud_msg->width, CV_8UC3);
    aist_utility::pointcloud_to_rgb(*cloud_msg, image.ptr<rgb_t>());
    detect_marker(*cloud_msg, *camera_info_msg, image);
}

template <class MSG> void
Detector::detect_marker(const MSG& msg, const camera_info_t& camera_info_msg,
			cv::Mat& image)
{
  // Convert camera_info to aruco camera parameters
    _camParam = rosCameraInfo2ArucoCamParams(camera_info_msg,
					     _useRectifiedImages);

  // Handle cartesian offset between stereo pairs.
  // See the sensor_msgs/CamaraInfo documentation for details.
    _rightToLeft.setIdentity();
    _rightToLeft.setOrigin(tf2::Vector3(
			       -camera_info_msg.P[3] / camera_info_msg.P[0],
			       -camera_info_msg.P[7] / camera_info_msg.P[5],
			       0.0));

  // Detect markers. Results will go into "markers"
    std::vector<aruco::Marker>	markers;
    _marker_detector.detect(image, markers, _camParam, _marker_size, false);

  // Show also the internal image resulting from the threshold operation
    publish_image(msg.header, _marker_detector.getThresholdedImage(),
		  _debug_pub);

    if (_marker_map.size() > 0)
    {
	std::vector<std::pair<point3_t, point3_t> >	pairs;

	for (const auto& marker : markers)
	{
	  // For each marker, draw info and its boundaries in the image.
	    marker.draw(image, cv::Scalar(0, 0, 255), 2);
	    aruco::CvDrawingUtils::draw3dAxis(
		image, const_cast<aruco::Marker&>(marker), _camParam);

	    const auto	corners = get_marker_corners(marker, msg, image);
	    const auto	i = _marker_map.getIndexOfMarkerId(marker.id);

	    if (i < 0 || corners.size() < 4)
		continue;

	    const auto&	markerinfo = _marker_map[i];
	    for (size_t j = 0; j < corners.size(); ++j)
		pairs.push_back(std::make_pair(markerinfo[j], corners[j]));
	}

	publish_transform(pairs.begin(), pairs.end(),
			  msg.header, _marker_frame, true);
    }
    else	// marker_id not specified.
    {
      // Publish all the detected markers.
	for (const auto& marker : markers)
	{
	  // For each marker, draw info and its boundaries in the image.
	    marker.draw(image, cv::Scalar(0, 0, 255), 2);
	    aruco::CvDrawingUtils::draw3dAxis(
		image, const_cast<aruco::Marker&>(marker), _camParam);

	    const auto	corners = get_marker_corners(marker, msg, image);
	    if (corners.size() < 4)
		continue;

	    std::vector<std::pair<point3_t, point3_t> >	pairs;
	    const auto	half_size = _marker_size/2;
	    pairs.push_back(std::make_pair(point3_t(-half_size, -half_size, 0),
					   corners[0]));
	    pairs.push_back(std::make_pair(point3_t(-half_size,  half_size, 0),
					   corners[1]));
	    pairs.push_back(std::make_pair(point3_t( half_size,  half_size, 0),
					   corners[2]));
	    pairs.push_back(std::make_pair(point3_t( half_size, -half_size, 0),
					   corners[3]));

	    publish_transform(pairs.begin(), pairs.end(), msg.header,
			      _marker_frame + std::to_string(marker.id),
			      false);
	}
    }

    publish_image(msg.header, image, _result_pub);
}

void
Detector::publish_image(const std_msgs::Header& header, const cv::Mat& image,
			const image_transport::Publisher& pub)
{
    using namespace	sensor_msgs;

    if (pub.getNumSubscribers() > 0)
	pub.publish(cv_bridge::CvImage(header, image_encodings::RGB8, image)
		    .toImageMsg());
}

template <class ITER> void
Detector::publish_transform(ITER begin, ITER end,
			    const std_msgs::Header& header,
			    const std::string& marker_frame, bool publish_pose)
{
    using element_t	= typename std::iterator_traits<ITER>::value_type
				      ::first_type::value_type;
    using matrix33_t	= cv::Matx<element_t, 3, 3>;
    using vector3_t	= cv::Matx<element_t, 3, 1>;
    using rigidity_t	= aist_utility::opencv::Rigidity<element_t, 3>;
    using similarity_t	= aist_utility::opencv::Similarity<element_t, 3>;

    try
    {
	matrix33_t	R;
	vector3_t	t;

	if (_useSimilarity)
	{
	    similarity_t	similarity;
	    const auto		residual = similarity.fit(begin, end);

	    ROS_DEBUG_STREAM("Fitted similarity transformation: scale = "
			     << similarity.s() << ", residual = " << residual);

	    R = similarity.R();
	    t = similarity.t() * (1/similarity.s());
	}
	else
	{
	    rigidity_t	rigidity;
	    const auto	residual = rigidity.fit(begin, end);

	    ROS_DEBUG_STREAM("Fitted rigid transformation: residual = "
			     << residual);

	    R = rigidity.R();
	    t = rigidity.t();
	}

	tf2::Transform	transform(tf2::Matrix3x3(R(0, 0), R(0, 1), R(0, 2),
						 R(1, 0), R(1, 1), R(1, 2),
						 R(2, 0), R(2, 1), R(2, 2)),
				  tf2::Vector3(t(0), t(1), t(2)));
	geometry_msgs::TransformStamped	transformMsg;
	transformMsg.header	    = header;
	transformMsg.child_frame_id = marker_frame;
	transformMsg.transform	    = tf2::toMsg(_rightToLeft * transform);
	_broadcaster.sendTransform(transformMsg);

	if (publish_pose && _pose_pub.getNumSubscribers() > 0)
	{
	    geometry_msgs::PoseStamped	poseMsg;
	    poseMsg.header = header;
	    tf2::toMsg(transform, poseMsg.pose);
	    _pose_pub.publish(poseMsg);
	}
    }
    catch (const std::exception& e)
    {
	ROS_WARN_STREAM(e.what());
    }
}

template <class MSG> std::vector<Detector::point3_t>
Detector::get_marker_corners(const aruco::Marker& marker,
			     const MSG& msg, cv::Mat& image) const
{
    using element_t	= point3_t::value_type;
    using point_t	= cv::Vec<element_t, 3>;
    using plane_t	= aist_utility::opencv::Plane<element_t, 3>;

    std::vector<point3_t>	corners(0);

    try
    {
      // Compute initial marker plane.
	std::vector<point3_t>	corner_points;
	for (const auto& corner : marker)
	{
	    const auto	point = at<element_t>(msg, corner.x, corner.y);

	    if (point(2) != element_t(0))
		corner_points.push_back(point);
	}

	plane_t	plane(corner_points.cbegin(), corner_points.cend());

      // Compute 2D bounding box of marker.
	const int	u0 = std::floor(std::min({marker[0].x, marker[1].x,
						  marker[2].x, marker[3].x}));
	const int	v0 = std::floor(std::min({marker[0].y, marker[1].y,
						  marker[2].y, marker[3].y}));
	const int	u1 = std::ceil( std::max({marker[0].x, marker[1].x,
						  marker[2].x, marker[3].x}));
	const int	v1 = std::ceil( std::max({marker[0].y, marker[1].y,
						  marker[2].y, marker[3].y}));

      // Select 3D points close to the initial plane within the bounding box.
	std::vector<point3_t>	points;
	for (auto v = v0; v <= v1; ++v)
	    for (auto u = u0; u <= u1; ++u)
	    {
		const auto	point = at<element_t>(msg, u, v);

		if (point(2) != element_t(0) &&
		    plane.distance(point) < _planarityTolerance)
		{
		    points.push_back(point);
		    image.at<rgb_t>(v, u).b = 0;
		}
	    }

      // Fit a plane to seleceted inliers.
	plane.fit(points.cbegin(), points.cend());

      // Compute 3D coordinates of marker corners and then publish.
	for (const auto& corner : marker)
	    corners.push_back(plane.cross_point(view_vector(corner.x,
							    corner.y)));
    }
    catch (const std::exception& e)
    {
	ROS_WARN_STREAM(e.what());
    }

    return corners;
}

template <class T> inline cv::Vec<T, 3>
Detector::view_vector(T u, T v) const
{
    cv::Mat_<cv::Point_<T> >	uv(1, 1), xy(1, 1);
    uv(0) = {u, v};
    cv::undistortPoints(uv, xy, _camParam.CameraMatrix, _camParam.Distorsion);

    return {xy(0).x, xy(0).y, T(1)};
}

template <class T> inline cv::Vec<T, 3>
Detector::at(const image_t& depth_msg, int u, int v) const
{
    if (u < 0 || u >= int(depth_msg.width) ||
	v < 0 || v >= int(depth_msg.height))
	return {T(0), T(0), T(0)};

    const auto	xyz = view_vector<T>(u, v);
    const auto	d   = val<T>(depth_msg, u, v);

    return {xyz[0]*d, xyz[1]*d, d};
}

template <class T> inline cv::Vec<T, 3>
Detector::at(const cloud_t& cloud_msg, int u, int v) const
{
    if (u < 0 || u >= int(cloud_msg.width) ||
	v < 0 || v >= int(cloud_msg.height))
	return {T(0), T(0), T(0)};

    sensor_msgs::PointCloud2ConstIterator<T>	xyz(cloud_msg, "x");
    xyz += (v * cloud_msg.width + u);

    return (!std::isnan(xyz[2]) && xyz[2] > T(0) ?
	    cv::Vec<T, 3>{xyz[0], xyz[1], xyz[2]} :
	    cv::Vec<T, 3>{T(0), T(0), T(0)});
}

template <class T, class MSG> cv::Vec<T, 3>
Detector::at(const MSG& msg, T u, T v) const
{
    const int	u0 = std::floor(u);
    const int	v0 = std::floor(v);
    const int	u1 = std::ceil(u);
    const int	v1 = std::ceil(v);
    for (auto vv = v0; vv <= v1; ++vv)
	for (auto uu = u0; uu <= u1; ++uu)
	{
	    const auto	xyz = at<T>(msg, uu, vv);
	    if (xyz[2] != T(0))
		return xyz;
	}

    return {T(0), T(0), T(0)};
}

/************************************************************************
*  class DetectorNodelet						*
************************************************************************/
class DetectorNodelet : public nodelet::Nodelet
{
  public:
			DetectorNodelet()				{}

    virtual void	onInit()					;

  private:
    boost::shared_ptr<Detector>	_node;
};

void
DetectorNodelet::onInit()
{
    NODELET_INFO("aist_aruco_ros::DetectorNodelet::onInit()");
    _node.reset(new Detector(getPrivateNodeHandle()));
}

}	// namespace aist_aruco_ros

PLUGINLIB_EXPORT_CLASS(aist_aruco_ros::DetectorNodelet, nodelet::Nodelet);
