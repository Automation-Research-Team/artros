/*!
 *  \file	ftsensor.cpp
 *  \brief	source file for a class for controlling force-torque sensors
 */
#include <yaml-cpp/yaml.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>		// for struct sockaddr_in
#include <arpa/inet.h>		// for inet_addr()
#include <netdb.h>		// for struct hostent, gethostbyname()
#include <cstdlib>		// for std::getenv()
#include <sys/stat.h>		// for mkdir()
#include <errno.h>
#include <fstream>
#include <Eigen/LU>
#include "ftsensor.h"

namespace aist_ftsensor
{
/************************************************************************
*  static functions							*
************************************************************************/
static const char*
splitd(const char* s, double& val)
{
    for (; *s; ++s)
	if (*s == '+' || *s == '-' || *s == '.' || isdigit(*s))
	{
	    char*	end;
	    val = strtod(s, &end);
	    return end;
	}

    throw std::runtime_error("No strings representing numeric values found.");
    return nullptr;
}

static ftsensor::vector3_t
get_vector3_param(const ros::NodeHandle& nh, const std::string& name)
{
    if (nh.hasParam(name))
    {
	std::vector<double>	v;
	nh.getParam(name, v);

	if (v.size() == 3)
	{
	    ftsensor::vector3_t	vec;
	    vec << v[0], v[1], v[2];
	    return vec;
	}
    }

    return ftsensor::vector3_t::Zero();
}

static ftsensor::quaternion_t
get_quaternion_param(const ros::NodeHandle& nh, const std::string& name)
{
    if (nh.hasParam(name))
    {
	std::vector<double>	v;
	nh.getParam(name, v);

	if (v.size() == 4)
	    return {v[3], v[0], v[1], v[2]};
    }

    return {1.0, 0.0, 0.0, 0.0};
}

//! Exterior product of two vectors.
template <class T, int M, int N> static Eigen::Matrix<T, M, N>
operator %(const ::Eigen::Matrix<T, M, 1>& x,
	   const ::Eigen::Matrix<T, N, 1>& y)
{
    ::Eigen::Matrix<T, M, N>	mat;
    for (size_t i = 0; i < M; ++i)
	for (size_t j = 0; j < N; ++j)
	    mat(i, j) = x(i) * y(j);
    return mat;
}

/************************************************************************
*  class ftsensor							*
************************************************************************/
ftsensor::ftsensor(const std::string& name, const Input input)
    :_nh(name),
     _input(input),
     _subscriber(input == Input::TOPIC ?
	    _nh.subscribe("/wrench_in", 100, &ftsensor::wrench_callback, this):
	    ros::Subscriber()),
     _publisher_org(_nh.advertise<wrench_t>("wrench_org", 100)),
     _publisher(_nh.advertise<wrench_t>("wrench", 100)),
     _socket(input == Input::SOCKET ? socket(AF_INET, SOCK_STREAM, 0): 0),
     _take_sample(_nh.advertiseService("take_sample",
				       &ftsensor::take_sample_callback, this)),
     _compute_calibration(_nh.advertiseService(
			      "compute_calibration",
			      &ftsensor::compute_calibration_callback, this)),
     _save_calibration(_nh.advertiseService(
			   "save_calibration",
			   &ftsensor::save_calibration_callback, this)),
     _listener(),
     _reference_frame(_nh.param<std::string>("reference_frame", "world")),
     _sensor_frame(_nh.param<std::string>("sensor_frame", "wrench_link")),
     _rate(_nh.param<int>("rate", 100)),
     _mg(G*_nh.param<double>("effector_mass", 0.0)),
     _q(get_quaternion_param(_nh, "rotation")),
     _f0(get_vector3_param(_nh, "force_offset")),
     _m0(get_vector3_param(_nh, "torque_offset")),
     _r(get_vector3_param(_nh, "mass_center")),
     _do_sample(false),
     _nsamples(0),
     _k_sum(vector3_t::Zero()),
     _f_sum(vector3_t::Zero()),
     _m_sum(vector3_t::Zero()),
     _kf_sum(matrix33_t::Zero()),
     _mm_sum(matrix33_t::Zero()),
     _ff_sum(matrix33_t::Zero()),
     _mf_sum(vector3_t::Zero()),
     _f_sqsum(0.0)
{
    ROS_INFO_STREAM("reference_frame=" << _reference_frame <<
		    ", sensor_frame=" << _sensor_frame << ", rate=" << _rate);
    ROS_INFO_STREAM("input=" << _input);
    ROS_INFO_STREAM("subscriber topic[" << _subscriber.getTopic() <<"]");
    ROS_INFO_STREAM("socket=" << _socket);

    if (_input == Input::SOCKET)
	up_socket();

    ROS_INFO_STREAM("aist_ftsensor started.");
}

ftsensor::~ftsensor()
{
    if (_input == Input::SOCKET)
	down_socket();
}

void
ftsensor::run()
{
    ros::Rate	rate(_rate);

    while (ros::ok())
    {
	tick();
	ros::spinOnce();
	rate.sleep();
    }
}

void
ftsensor::tick()
{
    if (_input != Input::SOCKET)
	return;

    std::array<char, 1024>	buf;
    const auto		nbytes = read(_socket, buf.data(), buf.size());
    if (nbytes < 0)
    {
	ROS_ERROR_STREAM("failed to read from socket.");
	throw;
    }
    buf[nbytes] = '\0';

    wrench_p	wrench(new wrench_t);
    wrench->header.stamp    = ros::Time::now();
    wrench->header.frame_id = _sensor_frame;

    const char*	s = buf.data();
    s = splitd(s, wrench->wrench.force.x);
    s = splitd(s, wrench->wrench.force.y);
    s = splitd(s, wrench->wrench.force.z);
    s = splitd(s, wrench->wrench.torque.x);
    s = splitd(s, wrench->wrench.torque.y);
    s = splitd(s, wrench->wrench.torque.z);

    wrench_callback(wrench);
}

double
ftsensor::rate() const
{
    return _rate;
}

void
ftsensor::wrench_callback(const wrench_p& wrench)
{
    try
    {
	_publisher_org.publish(wrench);

	transform_t	T;
	_listener.waitForTransform(_sensor_frame, _reference_frame,
				   wrench->header.stamp,
				   ros::Duration(1.0));
	_listener.lookupTransform(_sensor_frame, _reference_frame,
	 			  wrench->header.stamp, T);
	const auto	colz = T.getBasis().getColumn(2);
	vector3_t	k;			// direction of gravity force
	k << -colz.x(), -colz.y(), -colz.z();
	vector3_t	f;			// observed force
	f << wrench->wrench.force.x,
	     wrench->wrench.force.y,
	     wrench->wrench.force.z;
	vector3_t	m;			// observed torque
	m << wrench->wrench.torque.x,
	     wrench->wrench.torque.y,
	     wrench->wrench.torque.z;

	if (_do_sample)
	{
	    take_sample(k, f, m);
	    _do_sample = false;
	}

	const vector3_t	force  = _q*(f - _f0) - _mg*k;
	const vector3_t	torque = _q*(m - _m0) - _r.cross(_mg*k);
	
	wrench->header.frame_id = _sensor_frame;
	wrench->wrench.force.x  = force(0);
	wrench->wrench.force.y  = force(1);
	wrench->wrench.force.z  = force(2);
	wrench->wrench.torque.x = torque(0);
	wrench->wrench.torque.y = torque(1);
	wrench->wrench.torque.z = torque(2);

	_publisher.publish(wrench);
    }
    catch (const std::exception& err)
    {
	ROS_ERROR_STREAM(err.what());
    }
}

bool
ftsensor::take_sample_callback(std_srvs::Trigger::Request&  req,
			       std_srvs::Trigger::Response& res)
{
    _do_sample = true;

    res.success = true;
    res.message = "take_sample succeeded.";
    ROS_INFO_STREAM(res.message);

    return true;
}

bool
ftsensor::compute_calibration_callback(std_srvs::Trigger::Request&  req,
				       std_srvs::Trigger::Response& res)
{
    using namespace Eigen;
    
    if (_nsamples < 3)
    {
	res.message = "Not enough samples[" + std::to_string(_nsamples)
		    + "] for calibration!";
	res.success = false;
	ROS_ERROR_STREAM("(ftsensor) " << res.message);

	return true;
    }

  /*
   * Compute similarity transformation from external force to observed force.
   */
  // 1. Compute moment matrix.
    const vector3_t	k_avg = _k_sum /_nsamples;
    const vector3_t	f_avg = _f_sum /_nsamples;
    const matrix33_t	M     = _kf_sum/_nsamples - k_avg % f_avg;

  // 2. Apply SVD to moment matrix and correct resulting U and V
  //    so that they have positive determinant values.
    JacobiSVD<matrix33_t>	svd(M, ComputeFullU | ComputeFullV);
    matrix33_t	U = svd.matrixU();
    if (U.determinant() < 0)
	U.col(2) *= -1;
    matrix33_t	Vt = svd.matrixV().transpose();
    if (Vt.determinant() < 0)
	Vt.row(2) *= -1;

  // 3. Compute scale, rotation and translation components of similarity.
    _mg = (svd.singularValues()(0) + svd.singularValues()(1) +
	   svd.singularValues()(2)) / (1.0 - k_avg.squaredNorm());
    _q  = U * Vt;
    _f0 = f_avg -  _mg * (_q.inverse() * k_avg);

  /*
   * Compute transformation from external torque to observed torque.
   */
    const matrix33_t	A = (_f_sqsum/_nsamples - f_avg.squaredNorm())
			  * matrix33_t::Identity()
			  - _ff_sum/_nsamples + f_avg % f_avg;
    const vector3_t	b = _mf_sum/_nsamples - _f_sum.cross(f_avg)/_nsamples;
    const vector3_t	r = A.colPivHouseholderQr().solve(b);
    _m0 = _m_sum/_nsamples + r.cross(_f0 - f_avg);
    _r  = _q * r;
    
    res.success = true;
    res.message = "Successfully computed calibration.";
    ROS_INFO_STREAM(res.message);

    return true;
}

bool
ftsensor::save_calibration_callback(std_srvs::Trigger::Request&  req,
				    std_srvs::Trigger::Response& res)
{
    try
    {
	YAML::Emitter emitter;
	emitter << YAML::BeginMap;
	emitter << YAML::Key << "effector_mass"	<< YAML::Value << _mg/G;
	emitter << YAML::Key << "rotation"	<< YAML::Value << YAML::Flow
		<< YAML::BeginSeq
		<< _q.x() << _q.y() << _q.z() << _q.w()
		<< YAML::EndSeq;
	emitter << YAML::Key << "force_offset"	<< YAML::Value << YAML::Flow
		<< YAML::BeginSeq
		<< _f0(0) << _f0(1) << _f0(2)
		<< YAML::EndSeq;
	emitter << YAML::Key << "torque_offset"	<< YAML::Value << YAML::Flow
		<< YAML::BeginSeq
		<< _m0(0) << _m0(1) << _m0(2)
		<< YAML::EndSeq;
	emitter << YAML::Key << "mass_center"	<< YAML::Value << YAML::Flow
		<< YAML::BeginSeq
		<< _r(0) << _r(1) << _r(2)
		<< YAML::EndSeq;
	emitter << YAML::EndMap;

      // Read calibration file name from parameter server.
	const auto	calib_file = _nh.param<std::string>(
					"calib_file",
					std::string(getenv("HOME")) +
					"/.ros/aist_ftsensor/" +
					_nh.getNamespace() + ".yaml");

      // Open/create parent directory of the calibration file.
	const auto	dir = calib_file.substr(0,
						calib_file.find_last_of('/'));
	struct stat	buf;
	if (stat(dir.c_str(), &buf) && mkdir(dir.c_str(), S_IRWXU))
	    throw std::runtime_error("cannot create " + dir + ": "
						      + strerror(errno));

      // Open calibration file.
	std::ofstream	out(calib_file.c_str());
	if (!out)
	    throw std::runtime_error("cannot open " + calib_file + ": "
						    + strerror(errno));

      // Save calitration results.
	out << emitter.c_str() << std::endl;

	res.success = true;
	res.message = "save_calibration succeeded.";
	ROS_INFO_STREAM(res.message);
    }
    catch (const std::exception& err)
    {
	res.success = false;
	res.message = "save_calibration failed.";
	res.message += err.what();
	ROS_ERROR_STREAM(res.message);
    }

    return true;
}

void
ftsensor::take_sample(const vector3_t& k,
		      const vector3_t& f, const vector3_t& m)
{
    ++_nsamples;
    _k_sum   += k;
    _f_sum   += f;
    _m_sum   += m;
    _kf_sum  += k % f;
    _mm_sum  += m % m;
    _ff_sum  += f % f;
    _mf_sum  += m.cross(f);
    _f_sqsum += f.squaredNorm();
}

void
ftsensor::clear_samples()
{
    _nsamples = 0;
    _k_sum    = vector3_t::Zero();
    _f_sum    = vector3_t::Zero();
    _m_sum    = vector3_t::Zero();
    _kf_sum   = matrix33_t::Zero();
    _mm_sum   = matrix33_t::Zero();
    _ff_sum   = matrix33_t::Zero();
    _mf_sum   = vector3_t::Zero();
    _f_sqsum  = 0.0;
}
    
void
ftsensor::up_socket()
{
  // Check whether the socket is correctly opened.
    if (_socket < 0)
    {
	ROS_ERROR_STREAM("failed to open socket: " << strerror(errno));
	throw;
    }

  // Get hoastname and port from parameters.
    const auto	hostname = _nh.param<std::string>("hostname", "");
    const auto	port = _nh.param<int>("port", 63351);

  // Connect socket to hostname:port.
    auto	addr = inet_addr(hostname.c_str());
    if (addr == 0xffffffff)
    {
	const auto	h = gethostbyname(hostname.c_str());
	if (!h)
	{
	    ROS_ERROR_STREAM("unknown host name: " << hostname);
	    throw;
	}

	for (auto addr_ptr = (u_long**)h->h_addr_list; ; ++addr_ptr)
	{
	    if (!*addr_ptr)
		throw;
	    if (connect_socket(*(*addr_ptr), port))
		break;
	}
    }
    else
    {
	if (!connect_socket(addr, port))
	    throw;
    }
}

void
ftsensor::down_socket()
{
    if (_socket >= 0)
	close(_socket);
}

bool
ftsensor::connect_socket(u_long s_addr, int port)
{
    sockaddr_in	server;
    server.sin_family	   = AF_INET;
    server.sin_port	   = htons(port);
    server.sin_addr.s_addr = s_addr;
    ROS_INFO_STREAM("trying to connect socket to "
		    << inet_ntoa(server.sin_addr) << ':'
		    << port << "...");
    if (::connect(_socket, (sockaddr*)&server, sizeof(server)) == 0)
    {
	ROS_INFO_STREAM("succeeded.");
	return true;
    }
    else
    {
	ROS_ERROR_STREAM("failed: " << strerror(errno));
	return false;
    }
}

}	// namespace aist_ftsensor
