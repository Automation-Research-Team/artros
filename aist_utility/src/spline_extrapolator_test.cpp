/*
 *  \file	spline_extrapolator_test.cpp
 *  \brief	ROS tracker of aist_utility::PoseHeadAction type
 */
#include <ros/ros.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <aist_utility/Float32Stamped.h>
#include <aist_utility/spline_extrapolator.h>

namespace aist_utility
{
/************************************************************************
*  class SplineExtrapolatorTest						*
************************************************************************/
class SplineExtrapolatorTest
{
  public:
    using value_type	= float;

  public:
		SplineExtrapolatorTest()				;

    void	run()							;

  private:
    void	flt_cb(const Float32StampedConstPtr& flt)		;

  private:
    ros::NodeHandle			_nh;
    ros::Subscriber			_sub;
    const ros::Publisher		_pub;
    SplineExtrapolator<value_type, 2>	_extrapolator2;
    SplineExtrapolator<value_type, 3>	_extrapolator3;
    SplineExtrapolator<value_type, 4>	_extrapolator4;
};

SplineExtrapolatorTest::SplineExtrapolatorTest()
    :_nh("~"),
     _sub(_nh.subscribe("/in", 1, &SplineExtrapolatorTest::flt_cb, this)),
     _pub(_nh.advertise<geometry_msgs::Vector3Stamped>("out", 1))
{
}

void
SplineExtrapolatorTest::run()
{
    ros::Rate		rate(_nh.param<double>("rate", 100.0));
    ros::AsyncSpinner	spinner(8);
    spinner.start();

    while (ros::ok())
    {
	geometry_msgs::Vector3Stamped	vec;
	vec.header.stamp = ros::Time::now();
	vec.vector.x	 = _extrapolator2.pos(vec.header.stamp);
	vec.vector.y	 = _extrapolator3.pos(vec.header.stamp);
	vec.vector.z	 = _extrapolator4.pos(vec.header.stamp);
	_pub.publish(vec);

	rate.sleep();
    }

    spinner.stop();
    ros::waitForShutdown();

}

void
SplineExtrapolatorTest::flt_cb(const Float32StampedConstPtr& flt)
{
    const auto	now = ros::Time::now();

  //_extrapolator.update(flt->header.stamp, flt->x);
    _extrapolator2.update(now, flt->data);
    _extrapolator3.update(now, flt->data);
    _extrapolator4.update(now, flt->data);
}

}	// namepsace aist_utility

/************************************************************************
*  main function							*
************************************************************************/
int
main(int argc, char* argv[])
{
    ros::init(argc, argv, "spline_extrapolator_test");

    aist_utility::SplineExtrapolatorTest	spline_extrapolator_test;
    spline_extrapolator_test.run();

    return 0;
}
