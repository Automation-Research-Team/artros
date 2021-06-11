/*
 *  \file	ft300_driver.cpp
 *  \brief	source file of driver for Robotiq FT300 force-torque sensors
 */
#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
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
    using manager_t	= controller_manager::ControllerManager;

  public:
			ft300_driver()					;
    virtual		~ft300_driver()					;

    void		run()						;
    virtual void	read(const ros::Time&, const ros::Duration&)	;

  private:
    bool		up_socket()					;
    bool		connect_socket(u_long s_addr, int port)		;

  private:
    ros::NodeHandle	_nh;
    const double	_rate;
    const int		_socket;
    interface_t		_interface;
    double		_force[3];
    double		_torque[3];
};

ft300_driver::ft300_driver()
    :_nh("~"),
     _rate(_nh.param<int>("rate", 100)),
     _socket(::socket(AF_INET, SOCK_STREAM, 0)),
     _interface(),
     _force{0.0, 0.0, 0.0},
     _torque{0.0, 0.0, 0.0}
{
  // Check whether the socket is correctly opened.
    if (_socket < 0)
    {
	ROS_ERROR_STREAM("(ft300_driver) failed to open socket: "
			 << strerror(errno));
	throw;
    }

    if (!up_socket())
	throw;

  // Register hardware interface handle.
    const auto	frame_id = _nh.param<std::string>("frame_id", "wrench_link");
    _interface.registerHandle(handle_t(_nh.getNamespace() + "/wrench",
				       frame_id, &_force[0], &_torque[0]));
    registerInterface(&_interface);

    ROS_INFO_STREAM("(ft300_driver) ft300_driver started.");
}

ft300_driver::~ft300_driver()
{
    if (_socket >= 0)
	::close(_socket);
}

void
ft300_driver::run()
{
    ros::NodeHandle	nh;
    manager_t		manager(this, nh);
    ros::Rate		rate(_nh.param<double>("rate", 125.0));
    ros::AsyncSpinner	spinner(1);
    spinner.start();

    while (ros::ok())
    {
	read(ros::Time::now(), rate.cycleTime());
	manager.update(ros::Time::now(), rate.cycleTime());
	rate.sleep();
    }

    spinner.stop();
}

void
ft300_driver::read(const ros::Time&, const ros::Duration&)
{
    std::array<char, 1024>	buf;
    const auto			nbytes = ::read(_socket,
						buf.data(), buf.size());
    if (nbytes < 0)
    {
	ROS_ERROR_STREAM("(ftsensor) failed to read from socket: "
			 << strerror(errno));
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
}

bool
ft300_driver::up_socket()
{
  // Get hoastname and port from parameters.
    const auto	hostname = _nh.param<std::string>("hostname", "192.168.1.1");
    const auto	port	 = _nh.param<int>("port", 63351);

  // Connect socket to hostname:port.
    auto	addr = inet_addr(hostname.c_str());
    if (addr != 0xffffffff)
	return connect_socket(addr, port);

    const auto	h = gethostbyname(hostname.c_str());
    if (!h)
    {
	ROS_ERROR_STREAM("(ft300_driver) unknown host name: " << hostname);
	return false;
    }

    for (auto addr_ptr = (u_long**)h->h_addr_list; *addr_ptr; ++addr_ptr)
	if (connect_socket(*(*addr_ptr), port))
	    return true;

    return false;
}

bool
ft300_driver::connect_socket(u_long s_addr, int port)
{
    sockaddr_in	server;
    server.sin_family	   = AF_INET;
    server.sin_port	   = htons(port);
    server.sin_addr.s_addr = s_addr;
    ROS_INFO_STREAM("(ft300_driver) trying to connect socket to "
		    << inet_ntoa(server.sin_addr) << ':' << port << "...");
    if (::connect(_socket, (sockaddr*)&server, sizeof(server)) == 0)
    {
	ROS_INFO_STREAM("(ft300_driver) succeeded.");
	return true;
    }
    else
    {
	ROS_ERROR_STREAM("(ft300_driver) failed: " << strerror(errno));
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
	aist_ftsensor::ft300_driver	node;
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
