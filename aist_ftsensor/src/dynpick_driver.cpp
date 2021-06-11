/*
 *  \file	dynpick_driver.cpp
 *  \brief	source file of driver for Robotiq DYNPICK force-torque sensors
 */
#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/force_torque_sensor_interface.h>
#include <cstdio>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>

namespace aist_ftsensor
{
/************************************************************************
*  class dynpick_driver							*
************************************************************************/
class dynpick_driver : public hardware_interface::RobotHW
{
  public:
    using vector3_t	= std::array<double, 3>;

  private:
    using interface_t	= hardware_interface::ForceTorqueSensorInterface;
    using handle_t	= hardware_interface::ForceTorqueSensorHandle;
    using manager_t	= controller_manager::ControllerManager;
    
  public:
			dynpick_driver()				;
    virtual		~dynpick_driver()				;

    virtual void	read(const ros::Time&, const ros::Duration&)	;
    void		run()						;

  private:
    static vector3_t	get_gains(const ros::NodeHandle& nh,
				  const std::string& name,
				  const vector3_t& default_val)		;
    bool		set_tty()					;

  private:
    ros::NodeHandle	_nh;
    const double	_rate;
    const int		_fd;
    interface_t		_interface;
    vector3_t		_force_gains;
    vector3_t		_torque_gains;
    vector3_t		_force;
    vector3_t		_torque;
};

dynpick_driver::dynpick_driver()
    :_nh("~"),
     _rate(_nh.param<int>("rate", 100)),
     _fd(::open(_nh.param<std::string>("dev", "/dev/ttyUSB0").c_str(),
		O_RDWR | O_NOCTTY | O_NONBLOCK)),
     _interface(),
     _force_gains(get_gains(_nh, "force_gains",
			    {1.0/131.0, 1.0/131.0, 1.0/131.0})),
     _torque_gains(get_gains(_nh, "torque_gains",
			     {1.0/1310.0, 1.0/1310.0, 1.0/1310.0})),
     _force{0.0, 0.0, 0.0},
     _torque{0.0, 0.0, 0.0}
{
  // Check whether the socket is correctly opened.
    if (_fd < 0)
    {
	ROS_ERROR_STREAM("(dynpick_driver) failed to open tty: "
			 << strerror(errno));
	throw;
    }

    if (!set_tty())
	throw;

  // Register hardware interface handle.
    const auto	frame_id = _nh.param<std::string>("frame_id", "wrench_link");
    _interface.registerHandle(handle_t(_nh.getNamespace() + "/wrench",
				       frame_id, &_force[0], &_torque[0]));
    registerInterface(&_interface);

    ROS_INFO_STREAM("(dynpick_driver) dynpick_driver started.");
}

dynpick_driver::~dynpick_driver()
{
    if (_fd >= 0)
	::close(_fd);
}

void
dynpick_driver::read(const ros::Time&, const ros::Duration&)
{
    ssize_t	nbytes;
    if ((nbytes = ::write(_fd, "R", 1)) < 0)
    {
	ROS_ERROR_STREAM("(ftsensor) failed to write to tty: "
			 << strerror(errno));
	throw;
    }
    
    std::array<char, 1024>	buf;
    if ((nbytes = ::read(_fd, buf.data(), buf.size())) < 0)
    {
	ROS_ERROR_STREAM("(ftsensor) failed to read from tty: "
			 << strerror(errno));
	throw;
    }
    buf[nbytes] = '\0';

    int				tick;
    std::array<u_short, 6>	data;
    sscanf(buf.data(), "%1d%4hx%4hx%4hx%4hx%4hx%4hx",
	   &tick, &data[0], &data[1], &data[2], &data[3], &data[4], &data[5]);

    _force[0]  = _force_gains[0]  * data[0];
    _force[1]  = _force_gains[1]  * data[1];
    _force[2]  = _force_gains[2]  * data[2];
    _torque[0] = _torque_gains[0] * data[3];
    _torque[1] = _torque_gains[1] * data[4];
    _torque[2] = _torque_gains[2] * data[5];
}

void
dynpick_driver::run()
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

dynpick_driver::vector3_t
dynpick_driver::get_gains(const ros::NodeHandle& nh, const std::string& name,
			  const vector3_t& default_val)
{
    if (nh.hasParam(name))
    {
	std::vector<double>	v;
	nh.getParam(name, v);
	
	if (v.size() == 3)
	    return {v[0], v[1], v[2]};
    }

    return default_val;
}

bool
dynpick_driver::set_tty()
{
   termios      term;
   if (tcgetattr(_fd, &term) < 0)
       return false;

    bzero(&term, sizeof(term));

    term.c_cflag = B921600 | CS8 | CLOCAL | CREAD;
    term.c_iflag = IGNPAR;
    term.c_oflag = 0;
    term.c_lflag = 0;		// ICANON

    term.c_cc[VINTR]    = 0;	// Ctrl-c
    term.c_cc[VQUIT]    = 0;	// Ctrl-?
    term.c_cc[VERASE]   = 0;	// del
    term.c_cc[VKILL]    = 0;	// @
    term.c_cc[VEOF]     = 4;	// Ctrl-d
    term.c_cc[VTIME]    = 0;
    term.c_cc[VMIN]     = 0;
    term.c_cc[VSWTC]    = 0;	// '?0'
    term.c_cc[VSTART]   = 0;	// Ctrl-q
    term.c_cc[VSTOP]    = 0;	// Ctrl-s
    term.c_cc[VSUSP]    = 0;	// Ctrl-z
    term.c_cc[VEOL]     = 0;	// '?0'
    term.c_cc[VREPRINT] = 0;	// Ctrl-r
    term.c_cc[VDISCARD] = 0;	// Ctrl-u
    term.c_cc[VWERASE]  = 0;	// Ctrl-w
    term.c_cc[VLNEXT]   = 0;	// Ctrl-v
    term.c_cc[VEOL2]    = 0;	// '?0'

  //    tcflush(fd, TCIFLUSH);
    return tcsetattr(_fd, TCSANOW, &term) == 0;
}
    
}	// namepsace aist_ftsensor

/************************************************************************
*  global functions							*
************************************************************************/
int
main(int argc, char* argv[])
{
    ros::init(argc, argv, "dynpick0_driver");

    try
    {
	aist_ftsensor::dynpick_driver	node;
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
