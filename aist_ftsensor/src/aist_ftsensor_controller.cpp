/*
 *  \file	ftsensor_controller.cpp
 *  \brief	force-torque sensor controller with gravity compensation
 */
#include <ros/ros.h>
#include <controller_interface/controller.h>
#include <hardware_interface/force_torque_sensor_interface.h>
#include <realtime_tools/realtime_publisher.h>
#include <pluginlib/class_list_macros.hpp>
#include <geometry_msgs/WrenchStamped.h>
#include <aist_ftsensor/SetDecay.h>
#include <tf/transform_listener.h>
#include <std_srvs/Trigger.h>
#include <fstream>
#include <yaml-cpp/yaml.h>
#include <cstdlib>		// for std::getenv()
#include <sys/stat.h>		// for mkdir()
#include <Eigen/Dense>

namespace aist_ftsensor
{
/************************************************************************
*  static functions							*
************************************************************************/
template <class T, int M, int N> Eigen::Matrix<T, M, N>
operator %(const Eigen::Matrix<T, M, 1>& x,
	   const Eigen::Matrix<T, N, 1>& y)
{
    return x * y.transpose();
}

template <class T> Eigen::Matrix<T, 3, 3>
skew(const Eigen::Matrix<T, 3, 1>& vec)
{
    Eigen::Matrix<T, 3, 3>	mat;
    mat <<       0, -vec(2),  vec(1),
	    vec(2),	  0, -vec(0),
	   -vec(1),  vec(0),	   0;
    return mat;
}

template <class T, int M> std::ostream&
operator <<(std::ostream& out, const Eigen::Matrix<T, M, 1>& v)
{
    for (size_t i = 0; i < M; ++i)
	out << ' ' << v(i);
    return out;
}

template <class T> std::ostream&
operator <<(std::ostream& out, const std::vector<T>& v)
{
    for (const auto& elm : v)
	out << elm << std::endl;
    return out << std::endl;
}

/************************************************************************
*  class ForceTorqueSensorController					*
************************************************************************/
class ForceTorqueSensorController
    : public controller_interface::Controller<
		 hardware_interface::ForceTorqueSensorInterface>
{
  private:
    using interface_t	= hardware_interface::ForceTorqueSensorInterface;

    class Sensor
    {
      private:
	using handle_t		= hardware_interface::ForceTorqueSensorHandle;
	using publisher_t	= realtime_tools::RealtimePublisher<
					geometry_msgs::WrenchStamped>;
	using publisher_p	= std::shared_ptr<publisher_t>;
	using vector_t		= Eigen::Vector3d;
	using matrix_t		= Eigen::Matrix3d;
	using quaternion_t	= Eigen::Quaterniond;
	using ft_t		= Eigen::Matrix<double, 6, 1>;
	using transform_t	= tf::StampedTransform;

	constexpr static double	G = 9.80665;

      public:
		Sensor(interface_t* hw, ros::NodeHandle& root_nh,
		       const std::string& name, double pub_rate,
		       const tf::TransformListener& listener)		;

	void	starting(const ros::Time& time)				;
	void	update(const ros::Time& time,
		       const ros::Duration& period)			;
	void	stopping(const ros::Time& time)				{}
	void	save_calibration(std::ostream& out)		const	;

      private:
	bool	take_sample_cb(std_srvs::Trigger::Request&  req,
			       std_srvs::Trigger::Response& res)	;
	bool	compute_calibration_cb(std_srvs::Trigger::Request&  req,
				       std_srvs::Trigger::Response& res);
	bool	clear_samples_cb(std_srvs::Trigger::Request&  req,
				 std_srvs::Trigger::Response& res)	;
	bool	set_decay_cb(SetDecay::Request&  req,
			     SetDecay::Response& res)			;

	void	take_sample(const vector_t& k,
			    const vector_t& f, const vector_t& m)	;
	void	clear_samples()						;

	vector_t	vector_param(const std::string& name)	  const	;
	quaternion_t	quaternion_param(const std::string& name) const	;

      private:
	const handle_t			_hw_handle;
	const publisher_p		_pub_org;
	const publisher_p		_pub;
	const ros::Duration		_pub_interval;
	ros::Time			_last_pub_time;

      // ROS node stuffs
	ros::NodeHandle			_nh;
	const ros::ServiceServer	_take_sample;
	const ros::ServiceServer	_compute_calibration;
	const ros::ServiceServer	_clear_samples;
	const ros::ServiceServer	_set_decay;
	const tf::TransformListener&	_listener;

      // Variables retrieved from parameter server
	std::string			_robot_base_frame;
	double				_rate;
	double				_mg;		// effector mass
	quaternion_t			_q;		// rotation
	vector_t			_r;		// mass center
	vector_t			_f0;		// force offset
	vector_t			_m0;		// torque offset

      // Filtering stuffs
	double				_decay;
	ft_t				_ft;

      // Calibration stuffs
	bool				_do_sample;
	size_t				_nsamples;
	vector_t			_k_sum;
	vector_t			_f_sum;
	vector_t			_m_sum;
	double				_k_sqsum;
	matrix_t			_kf_sum;
	matrix_t			_km_sum;
	matrix_t			_mm_sum;

	std::ofstream			_fout;
    };

    using sensor_p	= std::shared_ptr<Sensor>;

  public:
			ForceTorqueSensorController()			{}

    virtual bool	init(interface_t* hw,
			     ros::NodeHandle &root_nh,
			     ros::NodeHandle& controller_nh)		;
    virtual void	starting(const ros::Time& time)			;
    virtual void	update(const ros::Time& time,
			       const ros::Duration& period)		;
    virtual void	stopping(const ros::Time& time)			;

  private:
    bool	save_calibration_cb(std_srvs::Trigger::Request&  req,
				    std_srvs::Trigger::Response& res)	;

  private:
    std::vector<sensor_p>	_sensors;
    const tf::TransformListener	_listener;
    std::string			_calib_file;
    ros::ServiceServer		_save_calibration;
};

bool
ForceTorqueSensorController::init(interface_t* hw,
				  ros::NodeHandle& root_nh,
				  ros::NodeHandle& controller_nh)
{
  // Get publishing period.
    double	pub_rate;
    if (!controller_nh.getParam("publish_rate", pub_rate))
    {
	ROS_ERROR_STREAM("(aist_ftsensor_controller) Parameter 'publish_rate' not set");
	return false;
    }

  // Setup sensors.
    for (const auto& name : hw->getNames())
	_sensors.push_back(sensor_p(new Sensor(hw, root_nh, name,
					       pub_rate, _listener)));

  // Get namespace for saving calibration.
  // Read calibration file name from parameter server.
    _calib_file = root_nh.param<std::string>("calib_file",
					     std::string(getenv("HOME")) +
					     "/.ros/aist_ftsensor" +
					     root_nh.getNamespace() + ".yaml");
    _save_calibration = root_nh.advertiseService(
			    "save_calibration",
			    &ForceTorqueSensorController::save_calibration_cb,
			    this);

    return true;
}

void
ForceTorqueSensorController::starting(const ros::Time& time)
{
    for (const auto& sensor : _sensors)
	sensor->starting(time);
}

void
ForceTorqueSensorController::update(const ros::Time& time,
				    const ros::Duration& period)
{
    for (const auto& sensor : _sensors)
	sensor->update(time, period);
}

void
ForceTorqueSensorController::stopping(const ros::Time& time)
{
    for (const auto& sensor : _sensors)
	sensor->stopping(time);
}

bool
ForceTorqueSensorController::save_calibration_cb(
    std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
    try
    {
      // Open/create parent directory of the calibration file.
	const auto	dir = _calib_file.substr(0,
						 _calib_file.find_last_of('/'));
	struct stat	buf;
	if (stat(dir.c_str(), &buf) && mkdir(dir.c_str(), S_IRWXU))
	    throw std::runtime_error("cannot create " + dir + ": "
						      + strerror(errno));

      // Open calibration file.
	std::ofstream	out(_calib_file.c_str());
	if (!out)
	    throw std::runtime_error("cannot open " + _calib_file + ": "
						    + strerror(errno));

      // Save calitration results.
	for (const auto& sensor : _sensors)
	    sensor->save_calibration(out);

	res.success = true;
	res.message = "save_calibration succeeded.";
	ROS_INFO_STREAM("(aist_ftsensor) " << res.message);
    }
    catch (const std::exception& err)
    {
	res.success = false;
	res.message = std::string("save_calibration failed: ") + err.what();
	ROS_ERROR_STREAM("(aist_ftsensor) " << res.message);
    }

    return true;
}

/************************************************************************
*  class ForceTorqueSensorController::Sensor				*
************************************************************************/
ForceTorqueSensorController::Sensor::Sensor(
    interface_t* hw, ros::NodeHandle& root_nh, const std::string& name,
    double pub_rate, const tf::TransformListener& listener)
    :_hw_handle(hw->getHandle(name)),
     _pub_org(new publisher_t(root_nh, name + "_org", 4)),
     _pub(new publisher_t(root_nh, name, 4)),
     _pub_interval(1.0/pub_rate),
     _last_pub_time(0),
     _nh(name),
     _take_sample(_nh.advertiseService("take_sample",
      				       &Sensor::take_sample_cb, this)),
     _compute_calibration(_nh.advertiseService("compute_calibration",
					       &Sensor::compute_calibration_cb,
					       this)),
     _clear_samples(_nh.advertiseService("clear_samples",
					 &Sensor::clear_samples_cb, this)),
     _set_decay(_nh.advertiseService("set_decay",
				     &Sensor::set_decay_cb, this)),
     _listener(listener),
     _robot_base_frame(_nh.param<std::string>("robot_base_frame", "world")),
     _mg(G*_nh.param<double>("effector_mass", 0.0)),
     _q(quaternion_param("rotation")),
     _r(vector_param("mass_center")),
     _f0(vector_param("force_offset")),
     _m0(vector_param("torque_offset")),
     _decay(_nh.param<double>("decay", 0.0)),
     _ft(ft_t::Zero()),
     _do_sample(false),
     _nsamples(0),
     _k_sum(vector_t::Zero()),
     _f_sum(vector_t::Zero()),
     _m_sum(vector_t::Zero()),
     _k_sqsum(0),
     _kf_sum(matrix_t::Zero()),
     _km_sum(matrix_t::Zero()),
     _mm_sum(matrix_t::Zero()),
     _fout()
{
    ROS_INFO_STREAM("(aist_ftsensor_controller) got sensor: " << name);
}

void
ForceTorqueSensorController::Sensor::starting(const ros::Time& time)
{
    _last_pub_time = time;
}

void
ForceTorqueSensorController::Sensor::update(const ros::Time& time,
					    const ros::Duration& period)
{
    if (_pub_interval > ros::Duration(0.0) &&
	_last_pub_time + _pub_interval < time)
    {
      // Get current force-torque values.
	ft_t	ft;
	ft << _hw_handle.getForce()[0],
	      _hw_handle.getForce()[1],
	      _hw_handle.getForce()[2],
	      _hw_handle.getTorque()[0],
	      _hw_handle.getTorque()[1],
	      _hw_handle.getTorque()[2];

	const auto& frame_id = _hw_handle.getFrameId();

	if (_pub_org->trylock())
	{
	  // Populate message.
	    _pub_org->msg_.header.stamp    = time;
	    _pub_org->msg_.header.frame_id = frame_id;
	    _pub_org->msg_.wrench.force.x  = ft[0];
	    _pub_org->msg_.wrench.force.y  = ft[1];
	    _pub_org->msg_.wrench.force.z  = ft[2];
	    _pub_org->msg_.wrench.torque.x = ft[3];
	    _pub_org->msg_.wrench.torque.y = ft[4];
	    _pub_org->msg_.wrench.torque.z = ft[5];

	    _pub_org->unlockAndPublish();
	}

      // Apply decay filter to input force-torque signal.
	(_ft *= _decay) += ft*(1.0 - _decay);

	if (_pub->trylock())
	{
	    vector_t	k;			// direction of gravity force
	    try
	    {
		transform_t	T;
		_listener.waitForTransform(_robot_base_frame, frame_id,
		 			   time, ros::Duration(0.1));
		_listener.lookupTransform(_robot_base_frame, frame_id,
					  time, T);
		const auto	rowz = T.getBasis().getRow(2);
		k << -rowz.x(), -rowz.y(), -rowz.z();
	    }
	    catch (const std::exception& err)
	    {
		_pub->unlock();
		ROS_ERROR_STREAM("(aist_ftsensor) " << err.what());
		return;
	    }

	  // Compute filtered force and torque.
	    const vector_t	f = _ft.head<3>();
	    const vector_t	m = _ft.tail<3>();

	    if (_do_sample)
	    {
		take_sample(k, f, m);
		_do_sample = false;
	    }

	  // Compensate force/torque offsets and gravity.
	    const vector_t force  = _q.inverse()*(f - _f0) - _mg*k;
	    const vector_t torque = _q.inverse()*(m - _m0) - _r.cross(_mg*k);

	  // We're actually publishing, so increment time.
	    _last_pub_time = _last_pub_time + _pub_interval;

	  // Populate message.
	    _pub->msg_.header.stamp    = time;
	    _pub->msg_.header.frame_id = frame_id;
	    _pub->msg_.wrench.force.x  = force(0);
	    _pub->msg_.wrench.force.y  = force(1);
	    _pub->msg_.wrench.force.z  = force(2);
	    _pub->msg_.wrench.torque.x = torque(0);
	    _pub->msg_.wrench.torque.y = torque(1);
	    _pub->msg_.wrench.torque.z = torque(2);

	    _pub->unlockAndPublish();
	}
    }
}

void
ForceTorqueSensorController::Sensor::save_calibration(std::ostream& out) const
{
    const auto	ns   = _nh.getNamespace();
    const auto	name = ns.substr(ns.find_last_of('/') + 1);

    YAML::Emitter emitter;
    emitter << YAML::BeginMap;
    emitter << YAML::Key << name << YAML::Value;
    emitter << YAML::BeginMap;
    emitter << YAML::Key << "effector_mass" << YAML::Value << _mg/G;
    emitter << YAML::Key << "rotation"	<< YAML::Value << YAML::Flow
	    << YAML::BeginSeq
	    << _q.x() << _q.y() << _q.z() << _q.w()
	    << YAML::EndSeq;
    emitter << YAML::Key << "force_offset" << YAML::Value << YAML::Flow
	    << YAML::BeginSeq
	    << _f0(0) << _f0(1) << _f0(2)
	    << YAML::EndSeq;
    emitter << YAML::Key << "torque_offset" << YAML::Value << YAML::Flow
	    << YAML::BeginSeq
	    << _m0(0) << _m0(1) << _m0(2)
	    << YAML::EndSeq;
    emitter << YAML::Key << "mass_center" << YAML::Value << YAML::Flow
	    << YAML::BeginSeq
	    << _r(0) << _r(1) << _r(2)
	    << YAML::EndSeq;
    emitter << YAML::Key << "decay" << YAML::Value << _decay;
    emitter << YAML::EndMap;
    emitter << YAML::EndMap;

    out << emitter.c_str() << std::endl;
}

bool
ForceTorqueSensorController::Sensor::take_sample_cb(
    std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
    _do_sample = true;
    if (!_fout.is_open())
	_fout.open("/tmp/aist_ftsensor_control.txt", std::ios_base::out);

    res.success = true;
    res.message = "take_sample succeeded.";
    ROS_INFO_STREAM("(aist_ftsensor) " << res.message);

    return true;
}

bool
ForceTorqueSensorController::Sensor::compute_calibration_cb(
    std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
    using namespace Eigen;

    if (_nsamples < 3)
    {
	res.message = "Not enough samples[" + std::to_string(_nsamples)
		    + "] for calibration!";
	ROS_ERROR_STREAM("(aist_ftsensor) " << res.message);
	return true;
    }

  // [0] Compute normal of the plane in which all the torque vectors lie.
    const vector_t	m_mean = _m_sum  / _nsamples;
    const matrix_t	mm_var = _mm_sum / _nsamples - m_mean % m_mean;
    SelfAdjointEigenSolver<matrix_t>	eigensolver(mm_var);
    vector_t		normal = eigensolver.eigenvectors().col(0);

    ROS_INFO_STREAM("(aist_ftsensor) RMS error in plane fitting: "
		    << std::sqrt(eigensolver.eigenvalues()(0)));

  // [1] Compute similarity transformation from gravity to observed torque.
  //   Note: Since rank(km_var) = 2, its third singular value is zero.
    const vector_t	k_mean = _k_sum  / _nsamples;
    const matrix_t	km_var = skew(normal) * (_km_sum / _nsamples -
						 k_mean % m_mean);
    JacobiSVD<matrix_t>	svd(km_var, ComputeFullU | ComputeFullV);

    matrix_t	Ut = svd.matrixU().transpose();
    if (Ut.determinant() < 0)
	Ut.row(2) *= -1;
    matrix_t	V = svd.matrixV();
    if (V.determinant() < 0)
	V.col(2) *= -1;
    _q = V * Ut;					// rotation

    const auto	k_var = _k_sqsum / _nsamples - k_mean.squaredNorm();
    const auto	scale = (svd.singularValues()(0) +
			 svd.singularValues()(1)) / k_var;
    _m0 = m_mean - scale * (_q * normal.cross(k_mean));	// torque offset

  // [2] Compute transformation from gravity to observed force.
    const vector_t	f_mean = _f_sum / _nsamples;
    const matrix_t 	kf_var = (_kf_sum / _nsamples - k_mean % f_mean);

  // If the effector mass value becomes negative, reverse the normal direction
  // and fix the rotation.
    if ((_q * kf_var).trace() < 0)
    {
	normal	  *= -1;	// Reverse the normal direction.
	Ut.row(0) *= -1;	// Reverse first two rows of Ut so that
	Ut.row(1) *= -1;	// the sign of SVD to be reversed while
				// keeping those of singular vlues.
	_q  = V * Ut;		// Recompute rotation.
    }
    _mg = (_q * kf_var).trace() / k_var;		// effector mass
    _r  = (scale / _mg) * normal;			// mass center
    _f0 = f_mean - _mg * (_q * k_mean);			// force offset

  // Evaluate residual error.
    // const auto	f_var = f_sqsum/_nsamples - f_avg.squaredNorm();
    // ROS_INFO_STREAM("(aist_ftsensor) force residual error = "
    // 		    << std::sqrt(f_var/k_var - _mg*_mg)
    // 		    << "(Newton)");

    res.success = true;
    res.message = "Successfully computed calibration.";
    ROS_INFO_STREAM("(aist_ftsensor) " << res.message);

    return true;
}

bool
ForceTorqueSensorController::Sensor::clear_samples_cb(
    std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
    clear_samples();

    res.success = true;
    res.message = "clear_samples succeeded.";
    ROS_INFO_STREAM("(aist_ftsensor) " << res.message);

    return true;
}

bool
ForceTorqueSensorController::Sensor::set_decay_cb(SetDecay::Request&  req,
						  SetDecay::Response& res)
{
    if (0.0 <= req.decay && req.decay < 1.0)
    {
	_decay = req.decay;
	res.success = true;
	ROS_INFO_STREAM("(aist_ftsensro) set decay value[" << _decay << "].");
    }
    else
    {
	res.success = false;
	ROS_INFO_STREAM("(aist_ftsensro) invalid decay value["
			<< req.decay << "] requested.");
    }

    return true;
}

void
ForceTorqueSensorController::Sensor::take_sample(const vector_t& k,
						 const vector_t& f,
						 const vector_t& m)
{
    ++_nsamples;
    _k_sum   += k;
    _f_sum   += f;
    _m_sum   += m;
    _k_sqsum += k.squaredNorm();
    _kf_sum  += k % f;
    _km_sum  += k % m;
    _mm_sum  += m % m;

    _fout << k.transpose() << std::endl;
    _fout << f.transpose() << std::endl;
    _fout << m.transpose() << std::endl << std::endl;
}

void
ForceTorqueSensorController::Sensor::clear_samples()
{
    _k_sum   = vector_t::Zero();
    _f_sum   = vector_t::Zero();
    _m_sum   = vector_t::Zero();
    _k_sqsum = 0;
    _kf_sum  = matrix_t::Zero();
    _km_sum  = matrix_t::Zero();
    _mm_sum  = matrix_t::Zero();

    if (_fout.is_open())
	_fout.close();
}

ForceTorqueSensorController::Sensor::vector_t
ForceTorqueSensorController::Sensor
			   ::vector_param(const std::string& name) const
{
    if (_nh.hasParam(name))
    {
	std::vector<double>	v;
	_nh.getParam(name, v);

	if (v.size() == 3)
	{
	    vector_t	vec;
	    vec << v[0], v[1], v[2];
	    return vec;
	}
    }

    return vector_t::Zero();
}

ForceTorqueSensorController::Sensor::quaternion_t
ForceTorqueSensorController::Sensor
			   ::quaternion_param(const std::string& name) const
{
    if (_nh.hasParam(name))
    {
	std::vector<double>	v;
	_nh.getParam(name, v);

	if (v.size() == 4)
	    return {v[3], v[0], v[1], v[2]};
    }

    return {1.0, 0.0, 0.0, 0.0};
}

}	// namespace aist_ftsensor

PLUGINLIB_EXPORT_CLASS(aist_ftsensor::ForceTorqueSensorController,
		       controller_interface::ControllerBase)
