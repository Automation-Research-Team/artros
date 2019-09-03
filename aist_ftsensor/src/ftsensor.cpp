/*!
 *  \file	ftsensor.cpp
 *  \brief	source file for a class for controlling force-torque sensors
 */
#include <ros/package.h>
#include <yaml-cpp/yaml.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>		// for struct sockaddr_in
#include <arpa/inet.h>		// for inet_addr()
#include <netdb.h>		// for struct hostent, gethostbyname()
#include <errno.h>
#include <fstream>

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
     _reference_frame("workspace_center"),
     _sensor_frame("ftsensor_wrench_link"),
     _rate(100),
     _get_sample(false),
     _mass(0),
     _f0(vector3_t::Zero()),
     _m0(vector3_t::Zero()),
     _r0(vector3_t::Zero()),
     _At_A(matrix44_t::Zero()),
     _At_b(vector4_t::Zero()),
     _Ct_C(matrix66_t::Zero()),
     _Ct_d(vector6_t::Zero())
{
    _nh.param<std::string>("reference_frame", _reference_frame,
			   "workspace_center");
    _nh.param<std::string>("sensor_frame", _sensor_frame,
			   "ftsensor_wrench_link");
    _nh.param<double>("rate", _rate, 100);

    ROS_INFO_STREAM("reference_frame=" << _reference_frame <<
		    ", sensor_frame=" << _sensor_frame << ", rate=" << _rate);

    ROS_INFO_STREAM("input=" << _input);
    ROS_INFO_STREAM("subscriber topic[" << _subscriber.getTopic() <<"]");
    ROS_INFO_STREAM("socket=" << _socket);

    if (_input == Input::SOCKET)
	up_socket();

    try
    {
	YAML::Node yaml_node = YAML::LoadFile(filepath());

	double effector_mass = yaml_node[KEY_EFFECTOR_MASS].as<double>();
	if (effector_mass > 0)
	    _mass = effector_mass;
	ROS_INFO_STREAM(KEY_EFFECTOR_MASS << "=" << _mass);

	std::vector<double> vec;
	vec = yaml_node[KEY_FORCE_OFFSET].as<std::vector<double> >();
	if (vec.size() >= 3)
	{
	    for (int i = 0; i < 3; i++)
		_f0(i) = vec[i];
	}
	ROS_INFO_STREAM(KEY_FORCE_OFFSET << "\n" << _f0);

	vec.clear();
	vec = yaml_node[KEY_TORQUE_OFFSET].as<std::vector<double> >();
	if (vec.size() >= 3)
	{
	    for (int i = 0; i < 3; i++)
		_m0(i) = vec[i];
	}
	ROS_INFO_STREAM(KEY_TORQUE_OFFSET << "\n" << _m0);

	vec.clear();
	vec = yaml_node[KEY_MASS_CENTER].as<std::vector<double> >();
	if (vec.size() >= 3)
	{
	    for (int i = 0; i < 3; i++)
		_r0(i) = vec[i];
	}
	ROS_INFO_STREAM(KEY_MASS_CENTER << "\n" << _r0);
    }
    catch (const std::exception& err)
    {
	ROS_ERROR_STREAM(err.what());
    }

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
    wrench->header.stamp = ros::Time::now();

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

bool
ftsensor::take_sample_callback(std_srvs::Trigger::Request  &req,
			       std_srvs::Trigger::Response &res)
{
    _get_sample = true;
    res.success = true;
    res.message = "take_sample succeeded.";
    ROS_INFO_STREAM(res.message);

    return true;
}

bool
ftsensor::compute_calibration_callback(std_srvs::Trigger::Request  &req,
				       std_srvs::Trigger::Response &res)
{
    const vector4_t	result_f = _At_A.inverse() * _At_b;
    _f0 << result_f(0), result_f(1), result_f(2);
    _mass  = -result_f(3)/G;

    const vector6_t result_t = _Ct_C.inverse() * _Ct_d;
    _m0 << result_t(0), result_t(1), result_t(2);
    const double	mg = _mass * G;
    _r0 << result_t(3)/mg, result_t(4)/mg, result_t(5)/mg;

#ifdef __MY_DEBUG__
    ROS_INFO_STREAM("_At_A\n" << _At_A);
    ROS_INFO_STREAM("_At_A(inverse)\n" << _At_A.inverse());
    ROS_INFO_STREAM("_At_b\n" << _At_b);
    ROS_INFO_STREAM("result(force)\n" << result_f);
    // ROS_INFO_STREAM("_Ct_C\n" << _Ct_C);
    // ROS_INFO_STREAM("_Ct_C(inverse)\n" << _Ct_C.inverse());
    // ROS_INFO_STREAM("_Ct_d\n" << _Ct_d);
    // ROS_INFO_STREAM("result(torque)\n" << result_t);
#endif /* __MY_DEBUG__ */

    _At_A = matrix44_t::Zero();
    _At_b = vector4_t::Zero();
    _Ct_C = matrix66_t::Zero();
    _Ct_d = vector6_t::Zero();

    res.success = true;
    res.message = "compute_calibration succeeded.";
    ROS_INFO_STREAM(res.message);

    return true;
}

bool
ftsensor::save_calibration_callback(std_srvs::Trigger::Request  &req,
				    std_srvs::Trigger::Response &res)
{
    try
    {
	YAML::Emitter emitter;
	emitter << YAML::BeginMap;
	emitter << YAML::Key << KEY_EFFECTOR_MASS << YAML::Value << _mass;
	emitter << YAML::Key << KEY_FORCE_OFFSET  << YAML::Value << YAML::Flow
		<< YAML::BeginSeq
		<< _f0(0) << _f0(1) << _f0(2)
		<< YAML::EndSeq;
	emitter << YAML::Key << KEY_TORQUE_OFFSET << YAML::Value << YAML::Flow
		<< YAML::BeginSeq
		<< _m0(0) << _m0(1) << _m0(2)
		<< YAML::EndSeq;
	emitter << YAML::Key << KEY_MASS_CENTER   << YAML::Value << YAML::Flow
		<< YAML::BeginSeq
		<< _r0(0) << _r0(1) << _r0(2)
		<< YAML::EndSeq;
	emitter << YAML::EndMap;

	std::ofstream f(filepath());
	f << emitter.c_str() << std::endl;

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
ftsensor::up_socket()
{
  // Check whether the socket is correctly opened.
    if (_socket < 0)
    {
	ROS_ERROR_STREAM("failed to open socket: " << strerror(errno));
	throw;
    }

  // Get hoastname and port from parameters.
    std::string	hostname;
    _nh.param<std::string>("hostname", hostname, "");
    int		port;
    _nh.param<int>("port", port, 63351);

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

void
ftsensor::wrench_callback(const const_wrench_p& wrench)
{
    try
    {
	transform_t	T;
	_listener.waitForTransform(_sensor_frame, _reference_frame,
				   wrench->header.stamp,
				   ros::Duration(1.0));
	_listener.lookupTransform(_sensor_frame, _reference_frame,
	 			  wrench->header.stamp, T);
	const auto	colz = T.getBasis().getColumn(2);
	vector3_t	k;
	k << colz.x(), colz.y(), colz.z();
	if (_get_sample)
	{
	    take_sample(k, wrench->wrench.force, wrench->wrench.torque);
	    _get_sample = false;
	}

	const vector3_t	force  = -_mass*G*k + _f0;
	const vector3_t	torque = -_r0.cross(_mass*G*k) + _m0;

	wrench_p	wrench_fixed(new wrench_t);
	wrench_fixed->header.stamp    = wrench->header.stamp;
	wrench_fixed->header.frame_id = _sensor_frame;
	wrench_fixed->wrench.force.x  = wrench->wrench.force.x  - force(0);
	wrench_fixed->wrench.force.y  = wrench->wrench.force.y  - force(1);
	wrench_fixed->wrench.force.z  = wrench->wrench.force.z  - force(2);
	wrench_fixed->wrench.torque.x = wrench->wrench.torque.x - torque(0);
	wrench_fixed->wrench.torque.y = wrench->wrench.torque.y - torque(1);
	wrench_fixed->wrench.torque.z = wrench->wrench.torque.z - torque(2);

	_publisher_org.publish(wrench);
	_publisher.publish(wrench_fixed);
    }
    catch (const std::exception& err)
    {
	ROS_ERROR_STREAM(err.what());
    }
}

void
ftsensor::take_sample(const vector3_t& k,
		      const geometry_msgs::Vector3& f,
		      const geometry_msgs::Vector3& m)
{
    // force
    matrix44_t At_A;
    At_A <<    1,    0,    0, k(0),
               0,    1,    0, k(1),
               0,    0,    1, k(2),
            k(0), k(1), k(2),    1;
    _At_A += At_A;

    vector4_t At_b;
    At_b << f.x, f.y, f.z, (k(0)*f.x + k(1)*f.y + k(2)*f.z);
    _At_b += At_b;

    // torque
    Eigen::Matrix<double, 3, 3> Ka; // antisymmetric matrix of k
    Ka <<     0, -k(2),  k(1),
           k(2),     0, -k(0),
          -k(1),  k(0),     0;
    Eigen::Matrix<double, 3, 3> Kat = Ka.transpose();
    Eigen::Matrix<double, 3, 3> Kat_Ka = Kat * Ka;

    matrix66_t Ct_C;
    Ct_C <<
	       1,        0,        0,    Ka(0, 0),    Ka(0, 1),    Ka(0, 2),
	       0,        1,        0,    Ka(1, 0),    Ka(1, 1),    Ka(1, 1),
	       0,        0,        1,    Ka(2, 0),    Ka(2, 1),    Ka(2, 2),
	Kat(0,0), Kat(0,1), Kat(0,2), Kat_Ka(0,0), Kat_Ka(0,1), Kat_Ka(0,2),
	Kat(1,0), Kat(1,1), Kat(1,2), Kat_Ka(1,0), Kat_Ka(1,1), Kat_Ka(1,2),
	Kat(2,0), Kat(2,1), Kat(2,2), Kat_Ka(2,0), Kat_Ka(2,1), Kat_Ka(2,2);
    _Ct_C += Ct_C;

    vector6_t Ct_d;
    Ct_d << m.x, m.y, m.z,
	    k(2)*m.y - k(1)*m.z, k(0)*m.z - k(2)*m.x, k(1)*m.x - k(0)*m.y;
    _Ct_d += Ct_d;

#ifdef __MY_DEBUG__
    ROS_INFO_STREAM("k\n" << k);
    ROS_INFO_STREAM("f\n" << f.x << " " << f.y << " " << f.z);
    ROS_INFO_STREAM("m\n" << m.x << " " << m.y << " " << m.z);
    ROS_INFO_STREAM(" At_A\n" <<  At_A);
    ROS_INFO_STREAM("_At_A\n" << _At_A);
    ROS_INFO_STREAM(" At_b\n" <<  At_b);
    ROS_INFO_STREAM("_At_b\n" << _At_b);
    // ROS_INFO_STREAM(" Ct_C\n" <<  Ct_C);
    // ROS_INFO_STREAM("_Ct_C\n" << _Ct_C);
    // ROS_INFO_STREAM(" Ct_d\n" <<  Ct_d);
    // ROS_INFO_STREAM("_Ct_d\n" << _Ct_d);

#if __MY_DEBUG__ > 1
    if (_to_dump)
    {
	try
	{
	    std::ofstream dump(DBG_DUMP_FILE, std::ios::app);
	    dump << f.x  << " " << f.y  << " " << f.z  << " "
	         << m.x  << " " << m.y  << " " << m.z  << " "
	         << k(0) << " " << k(1) << " " << k(2) << "\n";
	}
	catch (const std::exception& err)
	{
	    ROS_ERROR_STREAM(err.what());
	}
    }
#endif /* __MY_DEBUG__ > 1 */
#endif /* __MY_DEBUG__ */
}

std::string
ftsensor::filepath() const
{
    return ros::package::getPath("aist_ftsensor")
	 + "/config/" + _nh.getNamespace() + ".yaml";
}

}	// namespace aist_ftsensor
