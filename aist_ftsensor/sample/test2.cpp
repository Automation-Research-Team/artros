/*
 *  \file	ftsensor_controller.cpp
 *  \brief	source file of a class for controlling force-torque sensors
 */
#include <yaml-cpp/yaml.h>
#include <iterator>
#include <iostream>
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
class Sensor
{
  public:
    using vector_t	= Eigen::Vector3d;
    using matrix_t	= Eigen::Matrix3d;

  private:
    using quaternion_t	= Eigen::Quaterniond;

    constexpr static double	G = 9.80665;

  public:
		Sensor()						;

    void	take_sample(const vector_t& k,
			    const vector_t& f, const vector_t& m)	;
    bool	compute_calibration()					;
    void	save_calibration()				const	;
    vector_t	force( const vector_t& f, const vector_t& k)	const	;
    vector_t	torque(const vector_t& m, const vector_t& k)	const	;
    
  private:
  // Variables retrieved from parameter server
    double		_mg;		// effector mass
    quaternion_t	_q;		// rotation
    vector_t		_r;		// mass center
    vector_t		_f0;		// force offset
    vector_t		_m0;		// torque offset

  // Calibration stuffs
    size_t		_nsamples;
    vector_t		_k_sum;
    vector_t		_f_sum;
    vector_t		_m_sum;
    double		_k_sqsum;
    matrix_t		_kf_sum;
    matrix_t		_km_sum;
    matrix_t		_mm_sum;

};

Sensor::Sensor()
    :_mg(0),
     _q{1.0, 0.0, 0.0, 0.0},
     _r(vector_t::Zero()),
     _f0(vector_t::Zero()),
     _m0(vector_t::Zero()),
     _k_sum(vector_t::Zero()),
     _f_sum(vector_t::Zero()),
     _m_sum(vector_t::Zero()),
     _k_sqsum(0),
     _kf_sum(matrix_t::Zero()),
     _km_sum(matrix_t::Zero()),
     _mm_sum(matrix_t::Zero())
{
}

void
Sensor::take_sample(const vector_t& k, const vector_t& f, const vector_t& m)
{
    ++_nsamples;
    _k_sum   += k;
    _f_sum   += f;
    _m_sum   += m;
    _k_sqsum += k.squaredNorm();
    _kf_sum  += k % f;
    _km_sum  += k % m;
    _mm_sum  += m % m;
}

bool
Sensor::compute_calibration()
{
    using namespace Eigen;
    
    if (_nsamples < 3)
    {
	std::cerr << "Not enough samples[" << _nsamples
		  << "] for calibration!" << std::endl;
	return false;
    }

  // Compute normal of the plane in which all the torque vectors lie.
    const vector_t	m_mean = _m_sum  / _nsamples;
    const matrix_t	mm_var = _mm_sum / _nsamples - m_mean % m_mean;
    SelfAdjointEigenSolver<matrix_t>	eigensolver(mm_var);
    vector_t		normal = eigensolver.eigenvectors().col(0);

    std::cerr << "RMS error in plane fitting: "
	      << std::sqrt(eigensolver.eigenvalues()(0)) << std::endl;

  // Compute similarity transformation from gravity torque to observed torque.
  //   Note: Since rank(km_var) = 2, its third singular value is zero.
    const vector_t	k_mean = _k_sum  / _nsamples;
    const matrix_t	km_var = skew(normal) * (_km_sum / _nsamples -
						 k_mean % m_mean);
    JacobiSVD<matrix_t>	svd(km_var, ComputeFullU | ComputeFullV);
    matrix_t		Ut = svd.matrixU().transpose();
    std::cerr << "Singular values = " << svd.singularValues().transpose()
	      << std::endl;
    
    if (Ut.determinant() < 0)
	Ut.row(2) *= -1;
    matrix_t		V = svd.matrixV();
    if (V.determinant() < 0)
	V.col(2) *= -1;
    _q = V * Ut;
    
    const auto	k_var = _k_sqsum / _nsamples - k_mean.squaredNorm();
    const auto	scale = (svd.singularValues()(0) +
			 svd.singularValues()(1)) / k_var;
    _m0 = m_mean - scale * (_q * normal.cross(k_mean));	// torque offset

  // Compute 
    const vector_t	f_mean = _f_sum / _nsamples;
    const matrix_t 	kf_var = (_kf_sum / _nsamples - k_mean % f_mean);
    if ((_q * kf_var).trace() < 0)
    {
	normal	  *= -1;
	Ut.row(0) *= -1;
	Ut.row(1) *= -1;
	_q  = V * Ut;
    }
    _mg = (_q * kf_var).trace() / k_var;
    _r  = (scale / _mg) * normal;
    _f0 = f_mean - _mg * (_q * k_mean);

    return true;
}

void
Sensor::save_calibration() const
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

Sensor::vector_t
Sensor::force(const vector_t& f, const vector_t& k) const
{
  //return _q.inverse() * (f - _f0) - _mg * k;
    return _q * (_mg * k) + _f0 - f;
}

Sensor::vector_t
Sensor::torque(const vector_t& m, const vector_t& k) const
{
  //return _q.inverse() * (m - _m0) - _r.cross(_mg * k);
    return _q * (_r.cross(_mg * k)) + _m0 - m;
}

}	// namespace aist_ftsensor

int
main()
{
    aist_ftsensor::Sensor		sensor;
    std::vector<Eigen::Vector3d>	ks, fs, ms;
    
    while (std::cin)
    {
	Eigen::Vector3d	k, f, m;
	std::cin >> k(0) >> k(1) >> k(2)
		 >> f(0) >> f(1) >> f(2)
		 >> m(0) >> m(1) >> m(2);
	sensor.take_sample(k, f, m);

	ks.push_back(k);
	fs.push_back(f);
	ms.push_back(m);
    }

    sensor.compute_calibration();
    std::cerr << std::endl;
    sensor.save_calibration();

    std::cerr << std::endl;
    for (size_t i = 0; i < ks.size(); ++i)
	std::cerr << "force_err[" << i << "] = "
		  << sensor.force(fs[i], ks[i]).transpose() << std::endl;
    std::cerr << std::endl;
    for (size_t i = 0; i < ks.size(); ++i)
	std::cerr << "torque_err[" << i << "] = "
		  << sensor.torque(ms[i], ks[i]).transpose() << std::endl;
	
    return 0;
}
