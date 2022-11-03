/*
 *  \file	joint_state_extractor.cpp
 *  \brief	Utility tool for extracting states of specified joints
 */
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

namespace aist_controllers
{
class JointStateExtractor
{
  private:
    using joint_state_t	 = sensor_msgs::JointState;
    using joint_state_cp = sensor_msgs::JointStateConstPtr;

  public:
			JointStateExtractor()				;

    static void		run()						;

  private:
    void		joint_state_cb(const joint_state_cp& joint_state);

  private:
    ros::NodeHandle	_nh;
    ros::Subscriber	_joint_state_sub;
    ros::Publisher	_joint_state_pub;
    joint_state_t	_joint_state;
};

JointStateExtractor::JointStateExtractor()
    :_nh("~"),
     _joint_state_sub(_nh.subscribe("/joint_states", 1,
				    &JointStateExtractor::joint_state_cb,
				    this)),
     _joint_state_pub(_nh.advertise<joint_state_t>("joint_states", 1)),
     _joint_state()
{

    if (!_nh.getParam("joint_names", _joint_state.name))
	throw std::runtime_error("no joint names specified");

    const auto	njoints = _joint_state.name.size();
    _joint_state.position.resize(njoints);
    _joint_state.velocity.resize(njoints);
    _joint_state.effort  .resize(njoints);

    ROS_INFO_STREAM("(JointStateExtractor) started");
}

void
JointStateExtractor::run()
{
    ros::spin();
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

}	// namespace aist_controllers

int
main(int argc, char* argv[])
{
    ros::init(argc, argv, "joint_state_extractor");

    try
    {
	aist_controllers::JointStateExtractor	extractor;
	extractor.run();
    }
    catch (const std::exception& err)
    {
	std::cerr << err.what() << std::endl;
    }

    return 0;
}
