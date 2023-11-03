/*!
  \file		image_projector.cpp
  \author	Toshio Ueshiba
*/
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <sensor_msgs/CameraInfo.h>
#include <aist_visualization/TexturedMeshStamped.h>
#include <opencv2/imgproc.hpp>	// for cv::undistortPoints() (OpenCV3)
#include <opencv2/calib3d.hpp>	// for cv::undistortPoints() (OpenCV4)

namespace aist_visualization
{
/************************************************************************
*  class MeshGenerator							*
************************************************************************/
class MeshGenerator
{
  private:
    using camera_info_t	 = sensor_msgs::CameraInfo;
    using camera_info_cp = sensor_msgs::CameraInfoConstPtr;
    using transform_t	 = geometry_msgs::TransformStamped;
    using point_t	 = geometry_msgs::Point;
    using mesh_t	 = TexturedMeshStamped;

  public:
		MeshGenerator(ros::NodeHandle& nh)			;
		~MeshGenerator()					{}

  private:
    void	camera_info_cb(const camera_info_cp& cinfo)		;
    point_t	get_intersection(const cv::Point2d& image_point) const	;

  private:
    const ros::Subscriber		_camera_info_sub;
    const ros::Publisher		_mesh_pub;
    tf2_ros::Buffer			_tf2_buffer;
    const tf2_ros::TransformListener	_tf2_listener;

    transform_t				_Tsc;
    const size_t			_nsteps_u;
    const size_t			_nsteps_v;
    mesh_t				_mesh;
};

MeshGenerator::MeshGenerator(ros::NodeHandle& nh)
    :_camera_info_sub(nh.subscribe<camera_info_t>(
			  "/camera_info", 1,
			  &MeshGenerator::camera_info_cb, this)),
     _mesh_pub(nh.advertise<mesh_t>("mesh", 1)),
     _tf2_buffer(),
     _tf2_listener(_tf2_buffer),
     _Tsc(),
     _nsteps_u(nh.param<int>("nsteps_u", 10)),
     _nsteps_v(nh.param<int>("nsteps_v", 10)),
     _mesh()
{
  // Set ID of the screen frame.
    _Tsc.header.frame_id = nh.param<std::string>("screen_frame",
						 "conveyor_origin");

  // *** Set up mesh to be published. ***
  // 1. Allocate buffers for triangles, vertices and texture coordinates.
    _mesh.mesh.triangles.resize(2 * _nsteps_u * _nsteps_v);
    _mesh.mesh.vertices.resize((_nsteps_u + 1)*(_nsteps_v + 1));
    _mesh.u.resize(_mesh.mesh.vertices.size());
    _mesh.v.resize(_mesh.mesh.vertices.size());

  // 2. Set vertex indices.
    for (size_t j = 0; j < _nsteps_v; ++j)
	for (size_t i = 0; i < _nsteps_u; ++i)
	{
	    const auto	idx = i + j*(_nsteps_u + 1);
	    const auto	n   = i + j*_nsteps_u;
	    auto&	upper_triangle = _mesh.mesh.triangles[2*n];
	    auto&	lower_triangle = _mesh.mesh.triangles[2*n + 1];

	    upper_triangle.vertex_indices[0] = idx;
	    upper_triangle.vertex_indices[1] = idx + _nsteps_u + 2;
	    upper_triangle.vertex_indices[2] = idx + 1;
	    lower_triangle.vertex_indices[0] = idx;
	    lower_triangle.vertex_indices[1] = idx + _nsteps_u + 1;
	    lower_triangle.vertex_indices[2] = idx + _nsteps_u + 2;
	}

  // 3. Set texture coordinates.
    for (size_t j = 0; j <= _nsteps_v; ++j)
	for (size_t i = 0; i <= _nsteps_u; ++i)
	{
	    const auto	idx = i + j*(_nsteps_u + 1);

	    _mesh.u[idx] = double(i) / double(_nsteps_u);
	    _mesh.v[idx] = double(j) / double(_nsteps_v);
	}

    ROS_INFO_STREAM("(MeshGenerator) started");
}

void
MeshGenerator::camera_info_cb(const camera_info_cp& camera_info)
{
  // Get a transform from the camera frame to the screen frame.
    try
    {
	_Tsc = _tf2_buffer.lookupTransform(_Tsc.header.frame_id,
					   camera_info->header.frame_id,
					   camera_info->header.stamp,
					   ros::Duration(1.0));
    }
    catch (const tf2::TransformException& err)
    {
	ROS_ERROR_STREAM("MeshGenerator::camera_cb(): " << err.what());
	return;
    }

  // Compute 2D image coordinates mapped onto the mesh vertices.
    cv::Mat_<cv::Point2d>	uv(_mesh.mesh.vertices.size(), 1);
    auto			p = uv.begin();
    for (size_t idx = 0; idx < _mesh.mesh.vertices.size(); ++idx)
	*p++ = {_mesh.u[idx]*camera_info->width,
		_mesh.v[idx]*camera_info->height};

  // Extract camera matrix and lens distortion coefficients from camera_info.
    cv::Mat_<double>	K(3, 3), D(1, 4);
    std::copy_n(std::begin(camera_info->K), 9, K.begin());
    std::copy_n(std::begin(camera_info->D), 4, D.begin());

  // Transform the real image coordinates to the canonical image coordinates.
    cv::Mat_<cv::Point2d>	xy(_mesh.mesh.vertices.size(), 1);
    cv::undistortPoints(uv, xy, K, D);

  // Backproject canonical image points onto the screen.
    for (size_t idx = 0; idx < _mesh.mesh.vertices.size(); ++idx)
	_mesh.mesh.vertices[idx] = get_intersection(xy(idx));

    _mesh.header = camera_info->header;
    _mesh_pub.publish(_mesh);
}

MeshGenerator::point_t
MeshGenerator::get_intersection(const cv::Point2d& image_point) const
{
  // Create a view vector w.r.t. the camera frame.
    geometry_msgs::Vector3	vc;
    vc.x = image_point.x;
    vc.y = image_point.y;
    vc.z = 1;

  // Convert the view vector to the screen frame.
    geometry_msgs::Vector3	vs;
    tf2::doTransform(vc, vs, _Tsc);

  // Compute the intersection of the view vector and the screen.
    const auto			k = -_Tsc.transform.translation.z / vs.z;
    geometry_msgs::Point	p;
    p.x = k * vc.x;
    p.y = k * vc.y;
    p.z = k;

    return p;
}
}	// namespace aist_visualization

/************************************************************************
*  global functions							*
************************************************************************/
int
main(int argc, char* argv[])
{
    ros::init(argc, argv, "mesh_generator");

    try
    {
	ros::NodeHandle	nh("~");
	aist_visualization::MeshGenerator	mesh_generator(nh);

	ros::spin();
    }
    catch (const std::exception& err)
    {
	std::cerr << err.what() << std::endl;
	return 1;
    }

    return 0;
}
