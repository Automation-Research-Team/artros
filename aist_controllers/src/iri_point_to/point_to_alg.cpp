#include "point_to_alg.h"
#include <urdf/model.h>

PointToAlgorithm::PointToAlgorithm(void)
{
    pthread_mutex_init(&this->access_,NULL);
    this->fksolver=NULL;
    this->iksolver_vel=NULL;
    this->iksolver_pos=NULL;
    this->tree=NULL;
    this->chain=NULL;
    this->num_dof = 0;
    this->num_extra_dof = 0;
    this->total_dof = 0;
    this->max_attempts = this->config_.max_ik_attemps;
    this->max_ik_tolerance = this->config_.max_ik_tol;
    this->max_iter = this->config_.max_fk_attemps;
    this->max_fk_tolerance = this->config_.max_fk_tol;
    this->pointing_frame = "";
}

PointToAlgorithm::~PointToAlgorithm(void)
{
    pthread_mutex_destroy(&this->access_);
    if(this->fksolver!=NULL)
	delete this->fksolver;
    if(this->iksolver_vel!=NULL)
	delete this->iksolver_vel;
    if(this->iksolver_pos!=NULL)
	delete this->iksolver_pos;
    if(this->chain!=NULL)
	delete this->chain;
    if (this->tree != NULL)
	delete this->tree;
}

void
PointToAlgorithm::config_update(Config& config, uint32_t level)
{
    this->lock();

  // save the current configuration
    this->max_attempts = config.max_ik_attemps;
    this->max_ik_tolerance = config.max_ik_tol;
    this->max_iter = config.max_fk_attemps;
    this->max_fk_tolerance = config.max_fk_tol;

    if (this->config_.urdf_param.compare(config.urdf_param) != 0)
    {
	this->config_.urdf_param = config.urdf_param;
	set_tree();
	this->pointing_frame = "";
    }
    else if (this->config_.base_frame.compare(config.base_frame) != 0)
	this->pointing_frame = "";
    else if (this->max_attempts != this->config_.max_ik_attemps || this->max_ik_tolerance != this->config_.max_ik_tol ||
	     this->max_iter != this->config_.max_fk_attemps || this->max_fk_tolerance != this->config_.max_fk_tol)
	create_solvers();

    this->config_ = config;
  
    this->unlock();
}

// PointToAlgorithm Public API
void
PointToAlgorithm::print_info(void)
{
    if (this->chain == NULL)
    {
	ROS_ERROR("PointToAlgorithm::print_info-> No chain loaded.");
	return;
    }
    for (uint8_t i = 0; i < this->chain->getNrOfSegments(); i++)
    {
	KDL::Segment seg = this->chain->getSegment(i);
	ROS_INFO_STREAM("Segment number " << (int)i << ": name -> " << seg.getName());
	KDL::Joint joint = seg.getJoint();
	ROS_INFO_STREAM("Segment number " << (int)i << ": Joint name -> " << joint.getName());
	if (joint.getType() == KDL::Joint::RotAxis || joint.getType() == KDL::Joint::TransAxis)
	{
	    KDL::Vector v = joint.JointAxis();
	    ROS_INFO_STREAM("Segment number " << (int)i << ": Joint type -> " << (joint.getType() == KDL::Joint::RotAxis ? "Rotational Axis -> (": "Translational Axis -> (") << v[0] << ", " << v[1] << ", " << v[2] << ")" );
	    v = joint.JointOrigin();
	    ROS_INFO_STREAM("Segment number " << (int)i << ": Joint type -> Origin -> (" << v[0] << ", " << v[1] << ", " << v[2] << ")" );
	}
	else if (joint.getType() == KDL::Joint::None)
	{
	    KDL::Vector v = joint.JointAxis();
	    ROS_INFO_STREAM("Segment number " << (int)i << ": Joint type -> None -> (" << v[0] << ", " << v[1] << ", " << v[2] << ")" );
	    v = joint.JointOrigin();
	    ROS_INFO_STREAM("Segment number " << (int)i << ": Joint type -> Origin -> (" << v[0] << ", " << v[1] << ", " << v[2] << ")" );
	}
	else
	    ROS_INFO_STREAM("Segment number " << (int)i << ": Joint type -> " << joint.getTypeName());
	KDL::Frame frame = seg.getFrameToTip();
	KDL::Rotation rot = frame.M;
	KDL::Vector vec = frame.p;
	double a,d,alpha,theta;
	alpha = atan2(rot.data[7], rot.data[8]);
	theta = atan2(rot.data[3], rot.data[0]);
	d = vec[2];
	a = (rot.data[0] > rot.data[3]? vec[0]: vec[1])/(rot.data[0] > rot.data[3]? rot.data[0]: rot.data[3]);
	ROS_INFO_STREAM("Segment number " << (int)i << ": Frame -> a = " << a << "; alpha = " << alpha << "; d = " << d << "; theta = " << theta);

    }
}

bool
PointToAlgorithm::set_tree(void)
{
    if (this->tree != NULL)
	delete this->tree;
    this->tree = new KDL::Tree;

    ros::NodeHandle node;
    std::string urdf_model;
    node.param(this->config_.urdf_param, urdf_model, std::string());
    if (!kdl_parser::treeFromString(urdf_model, *this->tree)){
	ROS_ERROR("PointToAlgorithm::set_tree-> Failed to construct kdl tree");
	this->tree = NULL;
	return false;
    }
    return true;
}

bool
PointToAlgorithm::set_chain(void)
{
    if (this->chain != NULL)
	delete this->chain;

    this->chain = new KDL::Chain; 
    if (!this->tree->getChain(this->config_.base_frame, this->pointing_frame, *this->chain))
    {
	ROS_ERROR("PointToAlgorithm::set_chain-> No chain");
	this->chain = NULL;
	return false;
    }
    this->num_dof = this->chain->getNrOfJoints();
    add_extra_joints();
    this->total_dof = this->num_dof + this->num_extra_dof;
    if (!add_joint_limits())
	return false;
    create_solvers();
    return true;
}

bool
PointToAlgorithm::load_chain(std::string &pointing_frame)
{
    if (this->tree == NULL)
    {
	if (!set_tree())
	    return false;
	this->pointing_frame = "";
    }
    if (this->pointing_frame.compare(pointing_frame) != 0)
    {
	this->pointing_frame = pointing_frame;
	if (!set_chain())
	    return false;
    }
    return true;
}

void
PointToAlgorithm::create_solvers(void)
{
    if (this->chain == NULL)
    {
	ROS_ERROR("PointToAlgorithm::create_solvers-> Can't create solvers without a KDL::Chain");
	return;
    }
    if(this->fksolver!=NULL)
	delete this->fksolver;
    this->fksolver=new KDL::ChainFkSolverPos_recursive(*this->chain);
    if(this->iksolver_vel!=NULL)
	delete this->iksolver_vel;
    this->iksolver_vel=new KDL::ChainIkSolverVel_pinv(*this->chain);
    if(this->iksolver_pos!=NULL)
	delete this->iksolver_pos;
    this->iksolver_pos=new KDL::ChainIkSolverPos_NR_JL(*this->chain,this->q_min,this->q_max,*this->fksolver,*this->iksolver_vel,this->max_iter,this->max_fk_tolerance);
}

void
PointToAlgorithm::add_extra_joints(void)
{
    if (this->chain == NULL)
    {
	ROS_ERROR("PointToAlgorithm::add_extra_joints-> Can't add extra joints without a KDL::Chain");
	return;
    }
    this->num_extra_dof = 0;
    KDL::Rotation rot_kin(1, 0, 0, 0, 1, 0, 0, 0, 1);
    bool joints[6] = {false, false, false, false, false, false};
    for (uint8_t i = 0; i < this->chain->getNrOfSegments(); i++)
    {
	KDL::Segment seg = this->chain->getSegment(i);
	ROS_DEBUG_STREAM("PointToAlgorithm::add_extra_joints-> Segment number " << (int)i << ": name -> " << seg.getName());
	KDL::Joint joint = seg.getJoint();
	ROS_DEBUG_STREAM("PointToAlgorithm::add_extra_joints-> Segment number " << (int)i << ": Joint name -> " << joint.getName());
	KDL::Frame frame = seg.getFrameToTip();
	rot_kin = rot_kin*frame.M;

	if (joint.getType() != KDL::Joint::None)
	{
	    if (std::fabs(rot_kin.data[2]) > ROT_AXIS_THRESHOLD)
	    {
		joints[(joint.getType() < KDL::Joint::TransAxis? 0: 3)] = true;
		ROS_DEBUG_STREAM("PointToAlgorithm::add_extra_joints-> Found a " << (joint.getType() < KDL::Joint::TransAxis? "Rotational ": "Traslational ") << "joint with X axis");
	    }
	    else if (std::fabs(rot_kin.data[5]) > ROT_AXIS_THRESHOLD)
	    {
		joints[(joint.getType() < KDL::Joint::TransAxis? 1: 4)] = true;
		ROS_DEBUG_STREAM("PointToAlgorithm::add_extra_joints-> Found a " << (joint.getType() < KDL::Joint::TransAxis? "Rotational ": "Traslational ") << "joint with Y axis");
	    }
	    else if (std::fabs(rot_kin.data[8]) > ROT_AXIS_THRESHOLD)
	    {
		joints[(joint.getType() < KDL::Joint::TransAxis? 2: 5)] = true;
		ROS_DEBUG_STREAM("PointToAlgorithm::add_extra_joints-> Found a " << (joint.getType() < KDL::Joint::TransAxis? "Rotational ": "Traslational ") << "joint with Z axis");
	    }
	}
    }

    for (uint8_t i = 0; i < 6; i++)
    {
	if (!joints[i])
	{
	    KDL::Rotation r = rot_kin.Inverse();
	    uint8_t j = i%3;
	    this->num_extra_dof++;
	    this->chain->addSegment(KDL::Segment(KDL::Joint(KDL::Vector(0, 0, 0), KDL::Vector((std::fabs(r.data[0+j]) > ROT_AXIS_THRESHOLD? 1: 0), (std::fabs(r.data[3+j]) > ROT_AXIS_THRESHOLD? 1: 0), (std::fabs(r.data[6+j]) > ROT_AXIS_THRESHOLD? 1: 0)), (i < 3? KDL::Joint::RotAxis: KDL::Joint::TransAxis))));
	    rot_kin = rot_kin*this->chain->getSegment(this->chain->getNrOfSegments()-1).getFrameToTip().M;
	}
    }
}

bool
PointToAlgorithm::add_joint_limits(void)
{
    if (this->chain == NULL)
    {
	ROS_ERROR("PointToAlgorithm::add_joint_limits-> Can't add joint limits without a KDL::Chain");
	return false;
    }
    ros::NodeHandle node;
    urdf::Model model;
    if (!model.initParamWithNodeHandle(this->config_.urdf_param, node)) {
	ROS_ERROR("PointToAlgorithm::add_joint_limits-> Failed to parse urdf file");
	return false;
    }

    this->q_min.resize(this->total_dof);
    this->q_max.resize(this->total_dof);
    uint8_t aux = 0, i = 0;
    for (; i < this->num_dof; i++)
    {
	while (this->chain->getSegment(i+aux).getJoint().getType() == KDL::Joint::None)
	    aux++;
	urdf::JointConstSharedPtr joint = model.getJoint(this->chain->getSegment(i+aux).getJoint().getName());
	this->q_min(i) = joint->limits->lower;
	this->q_max(i) = joint->limits->upper;
	ROS_DEBUG_STREAM("PointToAlgorithm::add_joint_limits-> Joint-> " << this->chain->getSegment(i+aux).getJoint().getName() << " limits (" << this->q_min(i) << ", " << this->q_max(i) << ")");
    }
    for (; i < this->total_dof; i++)
    {
	while (this->chain->getSegment(i+aux).getJoint().getType() == KDL::Joint::None)
	    aux++;
	if (this->chain->getSegment(i+aux).getJoint().getType() > KDL::Joint::RotZ)
	{
	    this->q_min(i) = -100;
	    this->q_max(i) = 100;
	}
	else
	{
	    this->q_min(i) = -this->config_.extra_rot_limits;
	    this->q_max(i) = this->config_.extra_rot_limits;
	}
	ROS_DEBUG_STREAM("PointToAlgorithm::add_joint_limits-> Joint-> " << this->chain->getSegment(i+aux).getJoint().getName() << " limits (" << this->q_min(i) << ", " << this->q_max(i) << ")");
    }
    return true;
}

bool
PointToAlgorithm::get_inverse_kinematics(std::vector<double> &pos,
					 std::vector<double> &rot,
					 std::vector<double> &current_angles,
					 std::vector<double> &angles)
{
    if (this->iksolver_pos == NULL)
    {
	ROS_ERROR("PointToAlgorithm::get_inverse_kinematics-> Can't get inverse kinematics without a solver");
	return false;
    }
    int kinematics_status;
    KDL::JntArray q_init,joints;
    KDL::Frame cartpos_sol;
    KDL::Vector error;
    double max_error=0.0, desired_roll, desired_pitch, desired_yaw;
    double sol_roll, sol_pitch, sol_yaw; 
    unsigned int i=0,count=0;

    if (rot.size() != 3)
    {
	ROS_ERROR_STREAM("PointToAlgorithm::get_inverse_kinematics-> Rotation size != 3");
	return false;
    }
    if (pos.size() != 3)
    {
	ROS_ERROR_STREAM("PointToAlgorithm::get_inverse_kinematics-> Position size != 3");
	return false;
    }
    if (current_angles.size() != this->num_dof)
    {
	ROS_ERROR_STREAM("PointToAlgorithm::get_inverse_kinematics-> Current position size " << current_angles.size() << " != " << this->num_dof);
	return false;
    }
    KDL::Frame desired_frame(KDL::Rotation::RPY(rot[0],rot[1],rot[2]),
			     KDL::Vector(pos[0],pos[1],pos[2]));
    q_init.resize(this->chain->getNrOfJoints());
    joints.resize(this->chain->getNrOfJoints());

    do
    {
	count++;
	max_error=0.0;

	if (count == 1)
	{
	    uint8_t i = 0;
	    for(i = 0; i < this->num_dof; i++)
		q_init(i) = current_angles[i];
	    for(; i < this->total_dof; i++)
		q_init(i) = 0;
	}
	else
	{
	    uint8_t i = 0;
	    for(i = 0; i < this->total_dof; i++)
		q_init(i)=(rand()*(this->q_max(i) - this->q_min(i))/RAND_MAX) + this->q_min(i);
	  // for(i = 0; i < this->num_dof; i++)
	  //   q_init(i)=(rand()*(this->q_max(i) - this->q_min(i))/RAND_MAX) + this->q_min(i);
	  // for(; i < this->total_dof; i++)
	  //   q_init(i) = 0;
	}
	for(i=0;i<this->chain->getNrOfJoints();i++)
	{
	  // std::cout << q_init(i) << std::endl;
	    joints(i)=0.0;
	}

	kinematics_status=this->iksolver_pos->CartToJnt(q_init,desired_frame,joints);
      // kinematics_status=this->fksolver->JntToCart(joints,cartpos_sol);
	if(kinematics_status<0)
	{
	    ROS_ERROR("PointToAlgorithm::get_inverse_kinematics-> Impossible to find inverse solution1");
	  // return false;
	    max_error=10.0*this->max_ik_tolerance;
	    continue;
	}
	else
	{
	    kinematics_status=this->fksolver->JntToCart(joints,cartpos_sol);
	  // error=desired_frame.p-cartpos_sol.p;
	  // for(uint8_t i=0; i<3; i++)
	  //   if(fabs(error(i))>max_error)
	  //     max_error=fabs(error(i));

	    desired_frame.M.GetRPY(desired_roll, desired_pitch, desired_yaw);
	    cartpos_sol.M.GetRPY(sol_roll, sol_pitch, sol_yaw);
	    error[0] = desired_roll - sol_roll;
	    error[1] = desired_pitch - sol_pitch;
	    error[2] = desired_yaw - sol_yaw;
	    for(uint8_t i=0; i<3; i++)
		if(fabs(error(i))>max_error)
		    max_error=fabs(error(i)); 
	}
    } while (max_error>this->max_ik_tolerance && count<this->max_attempts);

    if(count<this->max_attempts)
    {
	angles.resize(this->num_dof);
	for(uint8_t i=0; i<this->num_dof; i++)
	    angles[i]=joints(i);
	return true;
    }    
    else
    {
	ROS_ERROR("PointToAlgorithm::get_inverse_kinematics-> Impossible to find inverse solution2");
	return false;
    }
}

void
PointToAlgorithm::get_joint_names(std::vector<std::string> &names)
{
    if (this->chain == NULL)
    {
	ROS_ERROR("PointToAlgorithm::get_joint_names-> Can't get joint names without a KDL::Chain");
	return ;
    }
    std::vector<std::string>().swap(names);
    uint8_t aux = 0, i = 0;
    for (; i < this->num_dof; i++)
    {
	while (this->chain->getSegment(i+aux).getJoint().getType() == KDL::Joint::None)
	    aux++;
	names.push_back(this->chain->getSegment(i+aux).getJoint().getName());
    }
}
