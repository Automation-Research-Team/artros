/*
 *  \file	spline_interpolator_test.cpp
 *  \brief	ROS tracker of aist_utility::PoseHeadAction type
 */
#include <ros/ros.h>
#include <aist_utility/Flt.h>
#include <aist_utility/spline_interpolator.h>

namespace aist_utility
{
/************************************************************************
*  class SplineInterpolatorTest						*
************************************************************************/
class SplineInterpolatorTest
{
  public:
    using value_type	= float;

  public:
		SplineInterpolatorTest()				;

    void	run()							;

  private:
    void	initialize(int half_order, double cutoff_frequency)	;
    void	flt_cb(const FltConstPtr& flt)				;

  private:
    ros::NodeHandle					_nh;
    ros::Subscriber					_sub;
    const ros::Publisher				_pub;
    aist_utility::SplineInterpolator<value_type>	_interpolator;
};

SplineInterpolatorTest::SplineInterpolatorTest()
    :_nh("~"),
     _sub(_nh.subscribe("/in", 1, &SplineInterpolatorTest::flt_cb, this)),
     _pub(_nh.advertise<aist_utility::Flt>("out", 1)),
     _interpolator()
{
}

void
SplineInterpolatorTest::run()
{
    ros::Rate		rate(_nh.param<double>("rate", 100.0));
    ros::AsyncSpinner	spinner(8);
    spinner.start();

    while (ros::ok())
    {
	aist_utility::Flt	flt;
	flt.header.stamp = ros::Time::now();
	flt.x		 = _interpolator.pos(flt.header.stamp);
	_pub.publish(flt);

	rate.sleep();
    }

    spinner.stop();
    ros::waitForShutdown();

}

void
SplineInterpolatorTest::flt_cb(const FltConstPtr& flt)
{
  //_interpolator.reset(flt->header.stamp, flt->x);
    _interpolator.update(ros::Time::now(), flt->x);
}

}	// namepsace aist_utility

/************************************************************************
*  main function							*
************************************************************************/
int
main(int argc, char* argv[])
{
    ros::init(argc, argv, "spline_interpolator_test");

    aist_utility::SplineInterpolatorTest	spline_interpolator_test;
    spline_interpolator_test.run();

    return 0;
}
