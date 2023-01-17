/*
 *  \file	spline_interpolator_test.cpp
 *  \brief	ROS tracker of aist_utility::PoseHeadAction type
 */
#include <ros/ros.h>
#include <geometry_msgs/Vector3Stamped.h>
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
    using value_type	= double;

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
    aist_utility::SplineInterpolator<value_type, 2>	_interpolator2;
    aist_utility::SplineInterpolator<value_type, 3>	_interpolator3;
    aist_utility::SplineInterpolator<value_type, 4>	_interpolator4;
};

SplineInterpolatorTest::SplineInterpolatorTest()
    :_nh("~"),
     _sub(_nh.subscribe("/in", 1, &SplineInterpolatorTest::flt_cb, this)),
     _pub(_nh.advertise<geometry_msgs::Vector3Stamped>("out", 1))
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
	geometry_msgs::Vector3Stamped	vec;
	vec.header.stamp = ros::Time::now();
	vec.vector.x	 = _interpolator2.pos(vec.header.stamp);
	vec.vector.y	 = _interpolator3.pos(vec.header.stamp);
	vec.vector.z	 = _interpolator4.pos(vec.header.stamp);
	_pub.publish(vec);

	rate.sleep();
    }

    spinner.stop();
    ros::waitForShutdown();

}

void
SplineInterpolatorTest::flt_cb(const FltConstPtr& flt)
{
    const auto	now = ros::Time::now();

  //_interpolator.update(flt->header.stamp, flt->x);
    _interpolator2.update(now, flt->x);
    _interpolator3.update(now, flt->x);
    _interpolator4.update(now, flt->x);
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
