/*
 *  \file	butterworth_lpf_test.cpp
 *  \brief	Test program for aist_utility::ButterworthLPF<T>
 */
#include <rclcpp/rclcpp.hpp>
#include <aist_msgs/msg/float32_stamped.hpp>
#include <aist_utility/butterworth_lpf.hpp>
#include <ddynamic_reconfigure2/ddynamic_reconfigure2.hpp>

namespace aist_utility
{
/************************************************************************
*  class ButterworthLPFTest						*
************************************************************************/
class ButterworthLPFTest : public rclcpp::Node
{
  public:
    using value_type	= float;

  private:
    using flt_t		= aist_msgs::msg::Float32Stamped;
    using flt_p		= flt_t::UniquePtr;

  public:
		ButterworthLPFTest(const rclcpp::NodeOptions& options)	;

  private:
    std::string	node_name()			const	{ return get_name(); }
    void	initialize(int half_order, double cutoff_frequency)	;
    void	flt_cb(flt_p in)				const	;

  private:
    ddynamic_reconfigure2::DDynamicReconfigure		_ddr;
    double						_rate;
    const rclcpp::Subscription<flt_t>::SharedPtr	_sub;
    const rclcpp::Publisher<flt_t>::SharedPtr		_pub;
    aist_utility::ButterworthLPF<value_type>		_lpf;
    mutable value_type					_x;
};

ButterworthLPFTest::ButterworthLPFTest(const rclcpp::NodeOptions& options)
    :rclcpp::Node("butterworth_lpf_test", options),
     _ddr(rclcpp::Node::SharedPtr(this)),
     _rate(_ddr.declare_read_only_parameter<double>("rate", 10.0)),
     _sub(create_subscription<flt_t>("/in", 1,
				     std::bind(&ButterworthLPFTest::flt_cb,
					       this, std::placeholders::_1))),
     _pub(create_publisher<flt_t>(node_name() + "/out", 1)),
     _lpf(2, 2.0/_rate),
     _x(0.0)
{
    using namespace	std::placeholders;

    _ddr.registerVariable<int>("lowpass_filter_half_order", _lpf.half_order(),
			       std::bind(&ButterworthLPFTest::initialize,
					 this, _1, _lpf.cutoff() * _rate),
			       "Half order of low pass filter", {1, 5});
    _ddr.registerVariable<double>("lowpass_filter_cutoff_frequency",
				  _lpf.cutoff() * _rate,
				  std::bind(&ButterworthLPFTest::initialize,
					    this, _lpf.half_order(), _1),
				  "Cutoff frequency", {0.1, 0.5*_rate});

    RCLCPP_INFO_STREAM(get_logger(), "started");
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
ButterworthLPFTest::flt_cb(flt_p in) const
{
    _x = in->data;

    flt_p	out(new flt_t);
    out->header = in->header;
    out->data   = _lpf.filter(in->data);
    _pub->publish(std::move(out));
  //RCLCPP_INFO_STREAM(get_logger(), "published");
}

}	// namepsace aist_utility

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(aist_utility::ButterworthLPFTest)
