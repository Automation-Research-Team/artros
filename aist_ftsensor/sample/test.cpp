/*
 *  \file	ftsensor_controller.cpp
 *  \brief	source file of a class for controlling force-torque sensors
 */
#include <yaml-cpp/yaml.h>
#include <iterator>
#include <iostream>
#include "Eigen.h"

namespace aist_ftsensor
{
/************************************************************************
*  static functions							*
************************************************************************/
static Eigen::Quaterniond
quaternion(const Eigen::Vector3d& v)
{
    const auto	theta2 = v.norm()/2;
    const auto	q = v.normalized() * std::sin(theta2);
    return {std::cos(theta2), q(0), q(1), q(2)};
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
class Sensor
{
  private:
    using vector_t	= Eigen::Vector3d;
    using matrix_t	= Eigen::Matrix3d;
    using quaternion_t	= Eigen::Quaterniond;

    constexpr static double	G = 9.80665;
    constexpr static size_t	NITER_MAX = 1000;

  public:
		Sensor()		;

    void	take_sample(const vector_t& k,
			    const vector_t& f, const vector_t& m)	;
    bool	compute_calibration()					;
    bool	compute_calibration2()					;
    void	save_calibration()					;

  private:
  // Variables retrieved from parameter server
    double			_mg;		// effector mass
    quaternion_t		_q;		// rotation
    vector_t			_r;		// mass center
    vector_t			_f0;		// force offset
    vector_t			_m0;		// torque offset

  // Calibration stuffs
    std::vector<vector_t>	_k;
    std::vector<vector_t>	_f;
    std::vector<vector_t>	_m;
};

Sensor::Sensor()
    :_mg(0),
     _q{1.0, 0.0, 0.0, 0.0},
     _r(vector_t::Zero()),
     _f0(vector_t::Zero()),
     _m0(vector_t::Zero()),
     _k(),
     _f(),
     _m()
{
}

void
Sensor::take_sample(const vector_t& k, const vector_t& f, const vector_t& m)
{
    _k.push_back(k);
    _f.push_back(f);
    _m.push_back(m);
}

bool
Sensor::compute_calibration()
{
    using namespace aist_utility::Eigen;

  // Check whether enough number of samples are available.
    const auto	nsamples = _k.size();

    if (nsamples < 3)
    {
	std::cerr << "Not enough samples[" << nsamples
		  << "] for calibration!" << std::endl;
	return false;
    }

    Plane<double, 3>	plane(_m.begin(), _m.end());
    const auto		r = plane.normal();

    std::cerr << "normal = " << r << std::endl;
    
    std::vector<std::pair<vector_t, vector_t> >	corres;
    for (size_t i = 0; i < _k.size(); ++i)
	corres.push_back(std::make_pair(r.cross(_k[i]), _m[i]));
    Similarity<double, 3>	similarity(corres.begin(), corres.end());

    _m0 = similarity.t();
    _q  = similarity.R();

  // Compute averages and deviations of gravity direction, force and torque.
    const vector_t	km = mean(_k.begin(), _k.end());
    auto		dk = _k;
    for (auto& v : dk)
	v -= km;
    const vector_t	fm = mean(_f.begin(), _f.end());
    auto		df = _f;
    for (auto& v : df)
	v -= fm;
    const vector_t	mm = mean(_m.begin(), _m.end());
    auto		dm = _m;
    for (auto& v : dm)
	v -= mm;

  //
    double	dfQdk_sum = 0.0, dk_sqsum = 0.0;
    for (size_t i = 0; i < nsamples; ++i)
    {
	dfQdk_sum += df[i].dot(_q * dk[i]);
	dk_sqsum  += dk[i].squaredNorm();
    }

    _mg = dfQdk_sum / dk_sqsum;
    _r  = (similarity.s()/_mg)*r;
    _f0 = fm - _mg*(_q*km);

  // Verify results.
  /*
    std::cerr << dm;
    for (size_t i = 0; i < nsamples; ++i)
    {
	const vector_t	err = _mg*(_r.cross(_q*_k[i])) + _m0 - _m[i];
	std::cerr << "torque_err[" << i << "] = " << err << std::endl;
    }

    std::cerr << dk;
    for (size_t i = 0; i < nsamples; ++i)
    {
	const vector_t	err = _mg*(_q*_k[i]) + _f0 - _f[i];
	std::cerr << "force_err[" << i << "] = " << err << std::endl;
    }
  */
    return true;
}

bool
Sensor::compute_calibration2()
{
    using namespace aist_utility::Eigen;

    const auto	nsamples = _k.size();

  // Compute averages and deviations of gravity direction, force and torque.
    const vector_t	km = mean(_k.begin(), _k.end());
    auto		dk = _k;
    for (auto& v : dk)
	v -= km;
    const vector_t	fm = mean(_f.begin(), _f.end());
    auto		df = _f;
    for (auto& v : df)
	v -= fm;
    const vector_t	mm = mean(_m.begin(), _m.end());
    auto		dm = _m;
    for (auto& v : dm)
	v -= mm;

  // Refine rotation.
    matrix_t	Qt;
    Qt <<  0,  0,  1,
	  -1,  0,  0,
	   0, -1,  0;
    _q = Qt;
    for (int niter = 0; ; )
    {
      // Compute gradient and Jacobian.
	vector_t	grad  = vector_t::Zero();
	matrix_t	jacob = matrix_t::Zero();
	for (size_t i = 0; i < nsamples; ++i)
	{
	    grad  += dk[i].cross(_q * dm[i]);
	    jacob += skew(dk[i]) * _q.toRotationMatrix() * skew(dm[i]);
	}

      // Solve for a vector representing small rotation.
	const vector_t	dtheta = jacob.colPivHouseholderQr().solve(grad);

      // Update rotation.
	const auto	dt = dtheta.norm()/2;
	const vector_t	dq = dtheta.normalized() * std::sin(dt);
	_q = quaternion_t(std::cos(dt), dq(0), dq(1), dq(2)) * _q;

	if (dt < 1.0e-7)
	    break;

	if (++niter == NITER_MAX)
	{
	    std::cerr << "Exceed max iteration number[" << niter
		      << "] for computing calibration!" << std::endl;
	    return false;
	}
    }

  // Compute rotation and mass center.

  // Compute effector mass and force/torque offsets.
    double	dk_sqsum = 0, fQk_sum = 0;
    for (size_t i = 0; i < nsamples; ++i)
    {
	dk_sqsum = dk[i].squaredNorm();
	fQk_sum  = (_q * df[i]).dot(dk[i]);
    }
    _mg = fQk_sum / dk_sqsum;
    _f0 = fm - _mg * (_q.inverse() * km);
    _m0 = mm - _mg * (_q.inverse() * _r.cross(fm));

    return true;
}

void
Sensor::save_calibration()
{
    YAML::Emitter emitter;
    emitter << YAML::BeginMap;
    emitter << YAML::Key << "effector_mass" << YAML::Value << _mg/G;
    emitter << YAML::Key << "rotation"	    << YAML::Value << YAML::Flow
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
    emitter << YAML::EndMap;

      // Save calitration results.
    std::cout << emitter.c_str() << std::endl;
}

}	// namespace aist_ftsensor

int
main()
{
    aist_ftsensor::Sensor	sensor;

    while (std::cin)
    {
	Eigen::Vector3d	k, f, m;
	std::cin >> k(0) >> k(1) >> k(2)
		 >> f(0) >> f(1) >> f(2)
		 >> m(0) >> m(1) >> m(2);
	sensor.take_sample(k, f, m);
    }

    sensor.compute_calibration();
    sensor.save_calibration();

    return 0;
}
