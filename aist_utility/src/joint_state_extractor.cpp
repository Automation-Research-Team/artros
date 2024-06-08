/*
 *  \file	joint_state_extractor.cpp
 *  \brief	Utility tool for extracting states of specified joints
 */
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <ddynamic_reconfigure2/ddynamic_reconfigure2.hpp>

namespace aist_utility
{
class JointStateExtractor : public rclcpp::Node
{
  private:
    using joint_state_t	 = sensor_msgs::msg::JointState;
    using joint_state_cp = sensor_msgs::msg::JointState::ConstSharedPtr;

  public:
		JointStateExtractor(const rclcpp::NodeOptions& options)	;

  private:
    std::string node_name()			const	{ return get_name(); }
    void	joint_state_cb(const joint_state_cp& joint_state)	;

  private:
    ddynamic_reconfigure2::DDynamicReconfigure		 _ddr;
    const rclcpp::Subscription<joint_state_t>::SharedPtr _joint_state_sub;
    const rclcpp::Publisher<joint_state_t>::SharedPtr	 _joint_state_pub;
    joint_state_t					 _joint_state;
};

JointStateExtractor::JointStateExtractor(const rclcpp::NodeOptions& options)
    :rclcpp::Node("joint_state_extractor", options),
     _ddr(rclcpp::Node::SharedPtr(this)),
     _joint_state_sub(create_subscription<joint_state_t>(
			  "/joint_states", 1,
			  std::bind(&JointStateExtractor::joint_state_cb,
				    this, std::placeholders::_1))),
     _joint_state_pub(create_publisher<joint_state_t>(
			  node_name() + "/joint_states", 1)),
     _joint_state()
{

    _joint_state.name = _ddr.declare_read_only_parameter<
			    std::vector<std::string> >("joint_names", {});

    const auto	njoints = _joint_state.name.size();
    _joint_state.position.resize(njoints);
    _joint_state.velocity.resize(njoints);
    _joint_state.effort  .resize(njoints);

    RCLCPP_INFO_STREAM(get_logger(), "started");
}

void
JointStateExtractor::joint_state_cb(const joint_state_cp& joint_state)
{
    for (size_t i = 0; i < _joint_state.name.size(); ++i)
    {
	const auto	iter = std::find(joint_state->name.begin(),
					 joint_state->name.end(),
					 _joint_state.name[i]);

	if (iter == joint_state->name.end())
	    return;

	const auto	j = iter - joint_state->name.begin();
	_joint_state.position[i] = joint_state->position[j];
	_joint_state.velocity[i] = joint_state->velocity[j];
	_joint_state.effort[i]   = joint_state->effort[j];
    }

    _joint_state.header = joint_state->header;
    _joint_state_pub->publish(_joint_state);
}

}	// namespace aist_utility

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(aist_utility::JointStateExtractor)
