/*
 *  \file	dynpick_driver.cpp
 *  \brief	source file of driver for Robotiq DYNPICK force-torque sensors
 */
#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/force_torque_sensor_interface.h>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <errno.h>

namespace aist_ftsensor
{
/************************************************************************
*  class dynpick_driver							*
************************************************************************/
class dynpick_driver : public hardware_interface::RobotHW
{
  private:
    using interface_t	= hardware_interface::ForceTorqueSensorInterface;
    using handle_t	= hardware_interface::ForceTorqueSensorHandle;
    using manager_t	= controller_manager::ControllerManager;
    using response_t	= std::array<char, 256>;
    using vector6_t	= std::array<double, 6>;

  public:
			dynpick_driver()				;
    virtual		~dynpick_driver()				;

    void		run()						;
    virtual void	read(const ros::Time&, const ros::Duration&)	;

  private:
    vector6_t		get_gains()				const	;
    static int		open_tty(const char* dev, u_int baud)		;
    void		put_command(const char* cmd)		const	;
    response_t		get_response()				const	;

  private:
    ros::NodeHandle	_nh;
    const double	_rate;
    const int		_fd;
    interface_t		_interface;
    const vector6_t	_gains;
    vector6_t		_ft;
};

dynpick_driver::dynpick_driver()
    :_nh("~"),
     _rate(_nh.param<int>("rate", 1000)),
     _fd(open_tty(_nh.param<std::string>("dev", "/dev/ttyUSB0").c_str(),
		  _nh.param<int>("baud", 921600))),
     _interface(),
     _gains(get_gains()),
     _ft{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}
{
  // Set number of points used for averaging filter
    const auto	avg_npoints = _nh.param<int>("avg_npoints", 1);
    switch (avg_npoints)
    {
      case 1:
	put_command("1F");
	break;
      case 2:
	put_command("2F");
	break;
      case 4:
	put_command("4F");
	break;
      case 8:
	put_command("8F");
	break;
      default:
	ROS_ERROR_STREAM("(dynpick_driver) unsupported avg_npoints value["
			 << avg_npoints << ']');
	throw;
    }

  // Register hardware interface handle.
    const auto	frame_id = _nh.param<std::string>("frame_id", "wrench_link");
    _interface.registerHandle(handle_t(_nh.getNamespace() + "/wrench",
				       frame_id, &_ft[0], &_ft[3]));
    registerInterface(&_interface);

    ROS_INFO_STREAM("(dynpick_driver) dynpick_driver started.");
}

dynpick_driver::~dynpick_driver()
{
    if (_fd >= 0)
	::close(_fd);
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

void
dynpick_driver::read(const ros::Time&, const ros::Duration&)
{
    put_command("R");
    const auto&	res = get_response();

    int				tick;
    std::array<short, 6>	data;
    sscanf(res.data(), "%1d%4hx%4hx%4hx%4hx%4hx%4hx",
	   &tick, &data[0], &data[1], &data[2], &data[3], &data[4], &data[5]);

    _ft[0] = _gains[0] * (data[0] - 8192);
    _ft[1] = _gains[1] * (data[1] - 8192);
    _ft[2] = _gains[2] * (data[2] - 8192);
    _ft[3] = _gains[3] * (data[3] - 8192);
    _ft[4] = _gains[4] * (data[4] - 8192);
    _ft[5] = _gains[5] * (data[5] - 8192);
}

dynpick_driver::vector6_t
dynpick_driver::get_gains() const
{
    put_command("p");
    const auto&	res = get_response();
    ROS_INFO_STREAM("(dynpick_driver) sensitivities: " << res.data());
    
    vector6_t	gains;
    sscanf(res.data(), "%lf,%lf,%lf,%lf,%lf,%lf",
	   &gains[0], &gains[1], &gains[2], &gains[3], &gains[4], &gains[5]);

    for (auto&& gain : gains)
	if (gain != 0.0)
	    gain = 1.0 / gain;

    return gains;
}

int
dynpick_driver::open_tty(const char* dev, u_int baud)
{
    const auto	fd = ::open(dev, O_RDWR | O_NOCTTY);
    if (fd < 0)
    {
	ROS_ERROR_STREAM("(dynpick_driver) failed to open tty: "
			 << strerror(errno));
	throw;
    }

    termios      term;
    if (tcgetattr(fd, &term) >= 0)
    {
	switch (baud)
	{
	  case 9600:
	    baud = B9600;
	    break;
	  case 19200:
	    baud = B19200;
	    break;
	  case 38400:
	    baud = B38400;
	    break;
	  case 57600:
	    baud = B57600;
	    break;
	  case 115200:
	    baud = B115200;
	    break;
	  case 921600:
	    baud = B921600;
	    break;
	  default:
	    ROS_ERROR_STREAM("(dynpick_driver) unsupported baud rate["
			     << baud << ']');
	    throw;
	}
	    
	bzero(&term, sizeof(term));

	term.c_cflag = baud | CS8 | CLOCAL | CREAD;
	term.c_iflag = IGNCR;		// Ignore CR
	term.c_oflag = 0;
	term.c_lflag = 0;

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
	if (tcsetattr(fd, TCSANOW, &term) == 0)
	    return fd;
    }

    ::close(fd);
    return -1;
}

void
dynpick_driver::put_command(const char* cmd) const
{
    if (::write(_fd, cmd, ::strlen(cmd)) < 0)
    {
	ROS_ERROR_STREAM("(ftsensor) failed to write to tty: "
			 << strerror(errno));
	throw;
    }
}

dynpick_driver::response_t
dynpick_driver::get_response() const
{
    response_t	res;
    ssize_t	nbytes = 0;
    do
    {
	const auto	n = ::read(_fd, &res[nbytes], res.size() - nbytes);
	if (n < 0)
	{
	    ROS_ERROR_STREAM("(ftsensor) failed to read from tty: "
			     << strerror(errno));
	    throw;
	}
	nbytes += n;
    } while (res[nbytes - 1] != '\n');
    res[nbytes - 1] = '\0';

    return res;
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
