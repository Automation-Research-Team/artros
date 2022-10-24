/*
 *  \file	pose_head_tracker.cpp
 *  \brief	ROS tracker of aist_controllers::PoseHeadAction type
 */
#include <ros/ros.h>
#include <aist_controllers/Flt.h>
#include <moveit_servo/butterworth_lpf.h>
#include <ddynamic_reconfigure/ddynamic_reconfigure.h>

namespace aist_controllers
{
/************************************************************************
*  class ButterworthLPFTest						*
************************************************************************/
class ButterworthLPFTest
{
  public:
    using value_type	= float;

  public:
		ButterworthLPFTest()					;

  private:
    void	initialize(int half_order, double cutoff_frequency)	;
    void	flt_cb(const FltConstPtr& flt)			const	;

  private:
    ros::NodeHandle				_nh;
    double					_rate;
    ros::Subscriber				_sub;
    const ros::Publisher			_pub;
    ddynamic_reconfigure::DDynamicReconfigure	_ddr;
    moveit_servo::ButterworthLPF<value_type>	_lpf;
    mutable value_type				_x;
};

ButterworthLPFTest::ButterworthLPFTest()
    :_nh("~"),
     _rate(_nh.param<double>("rate", 1000.0)),
     _sub(_nh.subscribe("/in", 1, &ButterworthLPFTest::flt_cb, this)),
     _pub(_nh.advertise<aist_controllers::Flt>("out", 1)),
     _lpf(2, 50.0/_rate),
     _x(0.0)
{
    _ddr.registerVariable<int>("lowpass_filter_half_order", _lpf.half_order(),
			       boost::bind(&ButterworthLPFTest::initialize,
					   this, _1, _lpf.cutoff() * _rate),
			       "Half order of low pass filter", 1, 5);
    _ddr.registerVariable<double>("lowpass_filter_cutoff_frequency",
				  _lpf.cutoff() * _rate,
				  boost::bind(&ButterworthLPFTest::initialize,
					      this, _lpf.half_order(), _1),
				  "Cutoff frequency", 0.5, 0.5*_rate);
    _ddr.publishServicesTopics();
}

void
ButterworthLPFTest::initialize(int half_order, double cutoff_frequency)
{
    std::cerr << "half_order=" << half_order
	      << ", cutoff_frequency=" << cutoff_frequency << std::endl;

    _lpf.initialize(half_order, cutoff_frequency/_rate);
    _lpf.reset(_x);
}

void
ButterworthLPFTest::flt_cb(const FltConstPtr& in) const
{
    _x = in->x;

    Flt	out;
    out.header = in->header;
    out.x      = _lpf.filter(in->x);
    _pub.publish(out);
}

}	// namepsace aist_controllers

/************************************************************************
*  main function							*
************************************************************************/
int
main(int argc, char* argv[])
{
    ros::init(argc, argv, "butterworth_lpf_test");

    const aist_controllers::ButterworthLPFTest	butterworth_lpf_test;
    ros::spin();

    return 0;
}
