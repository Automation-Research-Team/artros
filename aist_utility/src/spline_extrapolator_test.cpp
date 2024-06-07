/*
 *  \file	spline_extrapolator_test.cpp
 *  \brief	ROS tracker of aist_utility::PoseHeadAction type
 */
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <aist_msgs/msg/float32_stamped.hpp>
#include <aist_utility/spline_extrapolator.hpp>
#include <ddynamic_reconfigure2/ddynamic_reconfigure2.hpp>

namespace aist_utility
{
/************************************************************************
*  class SplineExtrapolatorTest						*
************************************************************************/
class SplineExtrapolatorTest : public rclcpp::Node
{
  public:
    using value_type	= float;

  private:
    using flt_t		= aist_msgs::msg::Float32Stamped;
    using flt_p		= flt_t::UniquePtr;
    using vector3_t	= geometry_msgs::msg::Vector3Stamped;
    using vector3_p	= vector3_t::UniquePtr;
    
  public:
		SplineExtrapolatorTest(const rclcpp::NodeOptions& options);

    void	tick()					;

  private:
    std::string node_name()			const	{ return get_name(); }
    void	flt_cb(flt_p flt)			;

  private:
    ddynamic_reconfigure2::DDynamicReconfigure		_ddr;
    const rclcpp::Subscription<flt_t>::SharedPtr	_sub;
    const rclcpp::Publisher<vector3_t>::SharedPtr	_pub;
    SplineExtrapolator<value_type, 2>			_extrapolator2;
    SplineExtrapolator<value_type, 3>			_extrapolator3;
    SplineExtrapolator<value_type, 4>			_extrapolator4;
    rclcpp::TimerBase::SharedPtr			_timer;
};

SplineExtrapolatorTest::SplineExtrapolatorTest(
    const rclcpp::NodeOptions& options)
    :rclcpp::Node("spline_extrapolator_test", options),
     _ddr(rclcpp::Node::SharedPtr(this)),
     _sub(create_subscription<flt_t>(
	      "/in", 1, std::bind(&SplineExtrapolatorTest::flt_cb, this,
				  std::placeholders::_1))),
     _pub(create_publisher<vector3_t>(node_name() + "out", 1)),
     _extrapolator2(get_clock()->now()),
     _extrapolator3(get_clock()->now()),
     _extrapolator4(get_clock()->now()),
     _timer(create_wall_timer(std::chrono::duration<double>(
				  1.0/_ddr.declare_read_only_parameter<double>(
					   "rate", 100.0)),
			      std::bind(&SplineExtrapolatorTest::tick, this)))
{
}

void
SplineExtrapolatorTest::tick()
{
    vector3_p	vec(new vector3_t);
    vec->header.stamp	= get_clock()->now();
  //vec->vector.x	= _extrapolator2.pos(vec->header.stamp);
    vec->vector.x	= _extrapolator2.xp();
    vec->vector.y	= _extrapolator3.pos(vec->header.stamp);
    vec->vector.z	= _extrapolator4.pos(vec->header.stamp);
    _pub->publish(std::move(vec));
}

void
SplineExtrapolatorTest::flt_cb(flt_p flt)
{
    const auto	now = get_clock()->now();

  //_extrapolator.update(flt->header.stamp, flt->x);
    _extrapolator2.update(now, flt->data);
    _extrapolator3.update(now, flt->data);
    _extrapolator4.update(now, flt->data);
}

}	// namepsace aist_utility

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(aist_utility::SplineExtrapolatorTest)
