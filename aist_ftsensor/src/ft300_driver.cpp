/*
 *  \file	ft300_driver.cpp
 *  \brief	source file of driver for Robotiq FT300 force-torque sensors
 */
#include <ros/ros.h>
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/force_torque_sensor_interface.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>		// for struct sockaddr_in
#include <arpa/inet.h>		// for inet_addr()
#include <netdb.h>		// for struct hostent, gethostbyname()
#include <errno.h>

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
*  class ft300_driver							*
************************************************************************/
class ft300_driver : public hardware_interface::RobotHW
{
  private:
    using interface_t	= hardware_interface::ForceTorqueSensorInterface;
    using handle_t	= hardware_interface::ForceTorqueSensorHandle;
    
  public:
		ft300_driver(const std::string& name)			;
    virtual	~ft300_driver()						;

    void	run()							;

  private:
    bool	connect_socket(u_long s_addr, int port)			;
    
  private:
    ros::NodeHandle	_nh;
    const double	_rate;
    const int		_socket;
    interface_t		_interface;
    double		_force[3];
    double		_torque[3];
};
    
ft300_driver::ft300_driver(const std::string& name)
    :_nh(name),
     _rate(_nh.param<int>("rate", 100)),
     _socket(::socket(AF_INET, SOCK_STREAM, 0))
{
  // Check whether the socket is correctly opened.
    if (_socket < 0)
    {
	ROS_ERROR("(ft300_driver) failed to open socket: %s", strerror(errno));
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
	    ROS_ERROR("(ft300_driver) unknown host name: %s", hostname.c_str());
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

  // Register hardware interface handle.
    const auto	frame_id = _nh.param<std::string>("frame_id", "wrench_link");
    _interface.registerHandle(handle_t(name, frame_id,
				       &_force[0], &_torque[0]));
    registerInterface(&_interface);
    
    ROS_INFO("(ft300_driver) ft300_driver started.");
}

ft300_driver::~ft300_driver()
{
    if (_socket >= 0)
	::close(_socket);
}

void
ft300_driver::run()
{
    ros::Rate	rate(_rate);

    while (ros::ok())
    {
	std::array<char, 1024>	buf;
	const auto		nbytes = ::read(_socket,
						buf.data(), buf.size());
	if (nbytes < 0)
	{
	    ROS_ERROR("(ftsensor) failed to read from socket.");
	    throw;
	}
	buf[nbytes] = '\0';

	const char*	s = buf.data();
	s = splitd(s, _force[0]);
	s = splitd(s, _force[1]);
	s = splitd(s, _force[2]);
	s = splitd(s, _torque[0]);
	s = splitd(s, _torque[1]);
	s = splitd(s, _torque[2]);

	// for (const auto& force : _force)
	//     std::cerr << ' ' << force;
	// std::cerr << ':';
	// for (const auto& torque : _torque)
	//     std::cerr << ' ' << torque;
	// std::cerr << std::endl;
	
	ros::spinOnce();
	rate.sleep();
    }
}

bool
ft300_driver::connect_socket(u_long s_addr, int port)
{
    sockaddr_in	server;
    server.sin_family	   = AF_INET;
    server.sin_port	   = htons(port);
    server.sin_addr.s_addr = s_addr;
    ROS_INFO("(ft300_driver) trying to connect socket to %s:%d...",
	     inet_ntoa(server.sin_addr), port);
    if (::connect(_socket, (sockaddr*)&server, sizeof(server)) == 0)
    {
	ROS_INFO("(ft300_driver) succeeded.");
	return true;
    }
    else
    {
	ROS_ERROR("(ft300_driver) failed: %s", strerror(errno));
	return false;
    }
}
}	// namepsace aist_ftsensor

/************************************************************************
*  global functions							*
************************************************************************/
int
main(int argc, char* argv[])
{
    ros::init(argc, argv, "ft3000_driver");

    try
    {
	aist_ftsensor::ft300_driver	node("~");
	node.run();
    }
    catch (const std::exception& err)
    {
	std::cerr << err.what() << std::endl;
	return 1;
    }
    catch (...)
    {
	std::cerr << "Unknown error." << std::endl;
	return 1;
    }

    return 0;
}
