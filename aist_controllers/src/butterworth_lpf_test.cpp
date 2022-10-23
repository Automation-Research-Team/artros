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
  private:
    using filter_t	= moveit_servo::ButterworthLPF<float>;

  public:
		ButterworthLPFTest()					;

    void	run()						const	;

  private:
    void	flt_cb(const FltConstPtr& flt)			const	;

  private:
    ros::NodeHandle				_nh;
    ros::Subscriber				_sub;
    const ros::Publisher			_pub;
    ddynamic_reconfigure::DDynamicReconfigure	_ddr;


    filter_t					_lpf;
};

ButterworthLPFTest::ButterworthLPFTest()
    :_nh("~"),
     _sub(_nh.subscribe("/in", 1, &ButterworthLPFTest::flt_cb, this)),
     _pub(_nh.advertise<aist_controllers::Flt>("out", 1)),
     _lpf(2, 0.5)
{
    _ddr.registerVariable<int>("lowpass_filter_half_order", _lpf.half_order(),
			       boost::bind(&filter_t::initialize,
					   _lpf, _1, _lpf.cutoff()),
			       "Half order of low pass filter", 1, 5);
    _ddr.registerVariable<double>("lowpass_filter_cutoff", _lpf.cutoff(),
				  boost::bind(&filter_t::initialize,
					      _lpf, _lpf.half_order(), _1),
				  "Normalized cutoff frequency", 0.0, 1.0);
    _ddr.publishServicesTopics();
}

void
ButterworthLPFTest::run() const
{
    ros::spin();
}

void
ButterworthLPFTest::flt_cb(const FltConstPtr& in) const
{
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

    aist_controllers::ButterworthLPFTest	butterworth_lpf_test;
    butterworth_lpf_test.run();

    return 0;
}
