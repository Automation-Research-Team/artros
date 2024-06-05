/*
 *  \file	joint_state_extractor.cpp
 *  \brief	Utility tool for extracting states of specified joints
 */
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace aist_utility
{
class JointStateExtractor : rclcpp::Node
{
  private:
    using joint_state_t	 = sensor_msgs::msg::JointState;
    using joint_state_cp = sensor_msgs::msg::JointStateConstPtr;

  public:
		JointStateExtractor(const rclcpp::NodeOptions& options)	;

  private:
    std::string fullname()	const	{ return get_fully_qualified_name(); }
    void	joint_state_cb(const joint_state_cp& joint_state)	;

  private:
    const rclcpp::Subscription<joint_state_t>::SharedPtr _joint_state_sub;
    const rclcpp::Publisher<joint_state_t>::SharedPtr	 _joint_state_pub;
    joint_state_t					 _joint_state;
};

JointStateExtractor::JointStateExtractor(const std::string& node_name,
					 const rclcpp::NodeOptions& options)
    :rclcpp::Node(node_name, options),
     _joint_state_sub(create_subscription<joint_state_t>(
			  "/joint_states", 1,
			  std::bind(&JointStateExtractor::joint_state_cb,
				    this, std::placeholders::_1))),
     _joint_state_pub(create_publisher<joint_state_t>(
			  fullname() + "joint_states", 1)),
     _joint_state()
{

    if (!_nh.getParam("joint_names", _joint_state.name))
	throw std::runtime_error("no joint names specified");

    const auto	njoints = _joint_state.name.size();
    _joint_state.position.resize(njoints);
    _joint_state.velocity.resize(njoints);
    _joint_state.effort  .resize(njoints);

    RCLCPP_INFO_STREAM(get_logger(), "(JointStateExtractor) started");
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
    _joint_state_pub.publish(_joint_state);
}

}	// namespace aist_utility

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(aist_utility::JointStateExtractor)
