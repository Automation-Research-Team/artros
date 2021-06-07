/*!
 *  \file	ftsensor.h
 *  \brief	header of a ROS node class for controlling force sensors
 */
#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>
#include <tf/transform_listener.h>
#include <std_srvs/Trigger.h>
#include <Eigen/Geometry>

namespace aist_ftsensor
{
/************************************************************************
*  class ftsensor							*
************************************************************************/
class ftsensor
{
  public:
    enum Input
    {
	TOPIC = 0,
	SOCKET,
    };

    using vector3_t	= Eigen::Vector3d;
    using matrix33_t	= Eigen::Matrix3d;
    using quaternion_t	= Eigen::Quaterniond;

  private:
    using wrench_t	= geometry_msgs::WrenchStamped;
    using wrench_p	= geometry_msgs::WrenchStampedPtr;
    using transform_t	= tf::StampedTransform;

    constexpr static double	G = 9.80665;

  public:
		ftsensor(const std::string& name,
			 const Input input=Input::TOPIC)		;
		~ftsensor()						;

    void	run()							;
    void	tick()							;
    double	rate()						const	;

  private:
    void	wrench_callback(const wrench_p& wrench)			;
    bool	take_sample_callback(std_srvs::Trigger::Request&  req,
				     std_srvs::Trigger::Response& res)	;
    bool	compute_calibration_callback(
			std_srvs::Trigger::Request&  req,
			std_srvs::Trigger::Response& res)		;
    bool	save_calibration_callback(
			std_srvs::Trigger::Request&  req,
			std_srvs::Trigger::Response& res)		;

    void	take_sample(const vector3_t& k,
			    const vector3_t& f, const vector3_t& m)	;
    void	clear_samples()						;
    void	up_socket()						;
    void	down_socket()						;
    bool	connect_socket(u_long hostname, int port)		;

  private:
    ros::NodeHandle		_nh;
    const Input			_input;
    const int			_socket;
    const ros::Subscriber	_subscriber;
    const ros::Publisher	_publisher_org;
    const ros::Publisher	_publisher;
    const ros::ServiceServer	_take_sample;
    const ros::ServiceServer	_compute_calibration;
    const ros::ServiceServer	_save_calibration;
    const tf::TransformListener	_listener;

  // Variables retrieved from parameter server
    std::string			_reference_frame;
    std::string			_sensor_frame;
    double			_rate;
    double			_mg;		// effector mass
    quaternion_t		_q;		// rotation
    vector3_t			_f0;		// force offset
    vector3_t			_m0;		// torque offset
    vector3_t			_r;		// mass center

  // Calibration stuffs
    bool			_do_sample;
    size_t			_nsamples;
    vector3_t			_k_sum;
    vector3_t			_f_sum;
    vector3_t			_m_sum;
    matrix33_t			_kf_sum;	// k % f
    matrix33_t			_mm_sum;	// m % m
    matrix33_t			_ff_sum;	// f % f
    vector3_t			_mf_sum;	// m ^ f
    double			_f_sqsum;	// f.f
};

}	// namespace aist_ftsensor
