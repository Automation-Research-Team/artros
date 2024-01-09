/*
 *  平成14-19年（独）産業技術総合研究所 著作権所有
 *  
 *  創作者：植芝俊夫
 *
 *  本プログラムは（独）産業技術総合研究所の職員である植芝俊夫が創作し，
 *  （独）産業技術総合研究所が著作権を所有する秘密情報です．著作権所有
 *  者による許可なしに本プログラムを使用，複製，改変，第三者へ開示する
 *  等の行為を禁止します．
 *  
 *  このプログラムによって生じるいかなる損害に対しても，著作権所有者お
 *  よび創作者は責任を負いません。
 *
 *  Copyright 2002-2007.
 *  National Institute of Advanced Industrial Science and Technology (AIST)
 *
 *  Creator: Toshio UESHIBA
 *
 *  [AIST Confidential and all rights reserved.]
 *  This program is confidential. Any using, copying, changing or
 *  giving any information concerning with this program to others
 *  without permission by the copyright holder are strictly prohibited.
 *
 *  [No Warranty.]
 *  The copyright holder or the creator are not responsible for any
 *  damages caused by using this program.
 *
 *  $Id$  
 */
/*!
  \file		CameraCalibrator.cc
  \brief	クラス#TU::CameraCalibratorの実装
*/
#include "CameraCalibrator.h"

namespace TU
{
/************************************************************************
*  static functions							*
************************************************************************/
//! 4つの係数と1つの変数から構成される3次式の値を求める．
/*!
  \param a0	0次項の係数
  \param a1	1次項の係数
  \param a2	2次項の係数
  \param a3	3次項の係数
  \param x	変数
  \return	a0 + a1*x + a2*x^2 + a3*x^3の値
*/
template <class T> static T
polynomial3(T a0, T a1, T a2, T a3, T x)
{
    return a0 + x * (a1 + x * (a2 + x * a3));
}

//! 与えられた3x3行列Gに対し，Gから単位行列の定数倍を引いたものがランク1となるような定数を求める．
/*!
  \f$\rank(\TUvec{G}{} - \mu\TUvec{I}{3\times 3}) = 1\f$となる\f$\mu\f$を計算する．
  \param G	3x3行列
  \return	求められた定数
*/
template <class T> static T
optimalEigenValue(const Eigen::Matrix<T, 3, 3>& G)
{
    const Eigen::Matrix<T, 1, 3>	a = G.row(1).cross(G.row(2)),
					b = G.row(2).cross(G.row(0)),
					c = G.row(0).cross(G.row(1));
    return -(a(1)*G(1, 0) + a(2)*G(2, 0) + b(0)*G(0, 1) + b(2)*G(2, 1) +
	     c(0)*G(0, 2) + c(1)*G(1, 2))
	   /(G(0, 1)*G(0, 1) + G(0, 2)*G(0, 2) +
	     G(1, 0)*G(1, 0) + G(1, 2)*G(1, 2) +
	     G(2, 0)*G(2, 0) + G(2, 1)*G(2, 1));
}

//! 同一次元を持つ2つのベクトルからそれらの要素の積から成る新しいベクトルを作る．
/*!
  \f$\TUvec{p}{} = [p_1, p_2,\ldots, p_d]^\top,~
  \TUvec{q}{} = [q_1, q_2,\ldots, q_d]^\top\f$に対して\f$d(d+1)/2\f$次元ベクトル
  \f[
  \begin{array}{ccccc}
  \TUvec{v}{} = [p_1 q_1, & p_1 q_2 + p_2 q_1, & p_1 q_3 + p_3 q_1,
  & \ldots, & p_1 q_d + p_d q_1, \\
  & p_2 q_2, & p_2 q_3 + p_3 q_2, & \ldots, & p_2 q_d + p_d q_2, \\
  & \ddots & \ddots & & \vdots \\
  & & & \ldots, & p_{d-1} q_d + p_d q_{d-1} \\
  & & & & p_d q_d]^\top
  \end{array}
  \f]
  を計算する．
  \param p	qと同一次元を持つベクトル
  \param q	pと同一次元を持つベクトル
  \return	計算されたベクトル
*/
template <class T, size_t N> static Eigen::Matrix<T, N*(N+1)/2, 1>
extpro(const Eigen::Matrix<T, N, 1>& p, const Eigen::Matrix<T, N, 1>& q)
{
    Eigen::Matrix<T, N*(N+1)/2, 1>	v;
    int					n = 0;
    for (size_t i = 0; i < N; ++i)
    {
	v(n++) = p(i)*q(i);
	for (size_t j = i+1; j < N; ++j)
	    v(n++) = p(i)*q(j) + p(j)*q(i);
    }
    return v;
}

template <class T, int R, int C> Eigen::Matrix<T, R, C>
swap_rows_and_cols(const Eigen::Matrix<T, R, C>& A)
{
    Eigen::Matrix<T, R, C>	B(A.rows(), A.cols());
    for (size_t i = 0; i < B.rows(); ++i)
	for (size_t j = 0; j < B.cols(); ++j)
	    B(i, j) = A(B.rows() - 1 - i, B.cols() - 1 - j);

    return B;
}
    
//! 射影空間において表現された参照平面行列から絶対円錐曲線の投影像(IAC)を計算する．
/*!
  \param Q	平面数個の参照平面行列を縦に並べた[3|4]x(3*平面数)行列
  \return	IACを表す3x3対称行列
*/
template <class T> static Eigen::Matrix<T, 3, 3>
computeIAC(const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>& Q)
{
    using vector3_type	= Eigen::Matrix<T, 3, 1>;
    using vector6_type	= Eigen::Matrix<T, 6, 1>;
    using matrix33_type	= Eigen::Matrix<T, 3, 3>;
    using matrix66_type	= Eigen::Matrix<T, 6, 6>;
    
    const size_t	nplanes = Q.cols() / 3;
    
  // Compute IAC(Image of Absolute Conic).
    matrix66_type	A;
    for (size_t j = 0; j < nplanes; ++j)
    {
	const vector3_type	p = Q.template block<3, 1>(0, 3*j);
	const vector3_type	q = Q.template block<3, 1>(0, 3*j+1);
	const vector6_type	a = extpro(p, p) - extpro(q, q);
	const auto		b = extpro(p, q);
	A += (a % a + b % b);
    }

    const auto	 evectors = Eigen::SelfAdjointEigenSolver<matrix66_type>(A)
			   .eigenvectors();
    vector6_type a;
    if (nplanes < 3)			// two reference planes.
	a = evectors(1, 5)*evectors.col(4) - evectors(1, 4)*evectors.col(5);
    else
	a = evectors.col(5);
    matrix33_type	omega;
    omega << a[0], a[1], a[2], a[1], a[3], a[4], a[2], a[4], a[5];

  // Make the lower-right element of omega.inv(), i.e. DIAC, becomes unity.
    omega *= (omega(0, 0)*omega(1, 1) -
	      omega(0, 1)*omega(1, 0)) / omega.determinant();

    return omega;
}

//! 特異値分解ができるように観測行列のスケールを調整する．
/*!
  \param W	(カメラ数x参照平面数)の2次元射影変換行列から成る観測行列を
		与えるとそのスケールを調整されたものが返される．  
*/
template <class T> static void
rescaleHomographies(Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>& W)
{
#ifdef _DEBUG
    using namespace	std;
    
    cerr << "*** Begin: TU::rescaleHomographies() ***" << endl;
#endif
    const auto	ncameras = W.rows()/3;
    const auto	nplanes  = W.cols()/3;
    const auto	H00inv   = W.template block<3, 3>(0, 0).inverse();
    for (size_t i = 1; i < ncameras; ++i)		// for each camera...
    {
      // Inter-image homography between camera 0 and i through plane 0.
	const auto	A = W.template block<3, 3>(3*i, 0) * H00inv;

	for (size_t j = 1; j < nplanes; ++j)	// for each plane...
	{
	    auto	Hij = W.template block<3, 3>(3*i, 3*j);
	    const Eigen::Matrix<T, 3, 3>
			G   = W.template block<3, 3>(0, 3*j) * Hij.inverse() * A;
	    const T	mu  = optimalEigenValue(G);
	    Hij *= mu;
#ifdef _DEBUG
	    const T	trG = G.trace(), trAdjG = trace(adjoint(G));
	    cerr << " H" << i << j
		 << ": a0 = " << polynomial3(-det(G), trAdjG, -trG, T(1), mu)
		 << ", a1 = " << polynomial3(trAdjG, -2*trG, T(3), T(0), mu)
		 << ", a2 = " << polynomial3(-trG, T(3), T(0), T(0), mu)
		 << ", mu = " << mu
		 << endl;
#endif
	}
    }
#ifdef _DEBUG
    cerr << "*** End:   TU::rescaleHomographies() ***\n" << endl;
#endif
}

//! スケール調整済みの観測行列をカメラの投影行列と参照平面行列に分解する．
/*!
  \param W		(カメラ数x参照平面数)の2次元射影変換行列から成る
			観測行列
  \param P		カメラ数個の投影行列を縦に並べた(3*カメラ数)x4行列が
			返される
  \param Q		平面数個の参照平面行列を横に並べた4x(3*平面数)行列が
			返される
  \param commonCenters	全カメラの投影中心が共通ならばtrue,
			そうでなければfalse
*/
template <class T> static void
factorHomographies(const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>& W,
		   Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>& P,
		   Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>& Q,
		   bool commonCenters)
{
    using namespace	Eigen;
    
    JacobiSVD	svd(W);
#ifdef _DEBUG
    using namespace		std;
    
    cerr << "*** Begin: TU::factorHomographies() ***\n"
	 << " singular values: " << svd.diagonal()(0, 5);
#endif
    P.setZero(W.rows(), 4);
    Q.setZero(4, W.cols());
    const size_t	rank = (commonCenters ? 3 : 4);
    for (size_t n = 0; n < rank; ++n)
    {
	for (size_t i = 0; i < P.rows(); ++i)
	    P(i, n) = svd.matrixU()(i, n);
	for (size_t j = 0; j < Q.cols(); ++j)
	    Q(n, j) = svd.singularValues(n) * svd.matrixV()(j, n);
    }
#ifdef _DEBUG
    cerr << "*** End:   TU::factorHomographies() ***\n" << endl;
#endif
}

//! カメラの投影行列と参照平面行列を射影空間からユークリッド空間における表現に変換する．
/*!
  \param P		カメラ数個の投影行列を縦に並べた行列を与えると，
			それがユークリッド空間における表現に直されて返される
  \param Q		平面数個の参照平面行列を横に並べた行列を与えると，
			それがユークリッド空間における表現に直されて返される
  \param scales		#computePlaneNormalizations で計算された各参照平面の
			2次元座標正規化変換のスケール
  \param commonCenters	全カメラの投影中心が共通ならばtrue, そうでなければfalse
*/ 
template <class T> static void
projectiveToMetric(Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>& P,
		   Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>& Q,
		   const std::vector<T>& scales, bool commonCenters)
{
    using namespace	Eigen;
    
    using vector4_type	= Matrix<T, 4, 1>;
    using matrix34_type	= Matrix<T, 3, 4>;
    using matrix44_type	= Matrix<T, 4, 4>;
    using vector_type	= Matrix<T, 1, Dynamic>;
#ifdef _DEBUG
    std::cerr << "*** Begin: TU::projectiveToMetric() ***" << std::endl;
#endif
    const auto		nplanes = Q.cols() / 3;
    
  // Fix WC(World Coordinates) to 0-th camera.
    JacobiSVD<matrix34_type>	svd(P.template block<3, 4>(0, 0));
    matrix44_type		Sinv;
    Sinv.template block<3, 4>(0, 0) = P.template block<3, 4>(0, 0);
    Sinv.row(3) = svd.matrixV().col(3).transpose();
#ifdef _DEBUG
    std::cerr << "  det(Sinv) = " << Sinv.determinant() << std::endl;
#endif    
    P = P * Sinv.inverse();
    Q = Sinv * Q;

  // Compute IAC(Image of Absolute Conic) of 0-th camera.
    const auto	omega = computeIAC(Q);
    const auto	K0inv = swap_rows_and_cols(
			    matrix33_t(
				swap_rows_and_cols(omega).llt().matrixL()));

    if (commonCenters)
    {
	P.block(0, 0, P.rows(), 3) = P.block(0, 0, P.rows(), 3)
				   * K0inv.inverse();
	Q.block(0, 0, 3, Q.cols()) = K0inv * Q.block(0, 0, 3, Q.cols());
    }
    else
    {
      // Compute PI(Plane at Inifinity).
	vector_type	b(3*nplanes), beta(nplanes);
	for (size_t j = 0; j < nplanes; ++j)
	{
	    const auto	Q3x2 = Q.template block<3, 2>(0, 3*j);
	    beta(j) = sqrt((Q.col(3*j  ).transpose()*omega*Q.col(3*j) +
			    Q.col(3*j+1).transpose()*omega*Q.col(3*j+1))/2.0);
#ifdef _DEBUG
	    std::cerr << "  " << j << "-th plane: beta = " << beta[j]
		      << std::endl;
#endif
	    b(3*j  ) = 0.0;
	    b(3*j+1) = 0.0;
	    b(3*j+2) = beta(j) * scales(j);
	}
	vector4_type	h = Q * b;		// PI(Plane at Infinity)
	solve(Q * Q.transpose(), h);

      // Compute transformation from projective to Euclidean coordinates.
	auto	TT = matrix44_type::Zero();
	TT.template block<3, 3>(0, 0) = K0inv;
	TT.row(3) = h;
	P = P * TT.inverse();
	Q = TT * Q;
	for (size_t j = 0; j < nplanes; ++j)
	    Q.template block<4, 3>(0, 3*j) /= beta[j];
    }
    
  // Ensure the left 3x3 part of camera matrices to have positive determinant.
    const size_t	ncameras = P.rows() / 3;
    for (size_t i = 0; i < ncameras; ++i)
	if (P.template block<3, 3>(3*i, 0).determinant() < 0.0)
	    P.template block<3, 4>(3*i, 0) *= -1.0;

  // Ensure that the plane origins are in front of the cameras.
    size_t	nnegatives = 0;
    for (size_t i = 0; i < ncameras; ++i)
    {
	const auto	r3 = P.template block<1, 3>(3*i+2, 0);
	
	for (size_t j = 0; j < nplanes; ++j)
	{
	    if (r3 * Q.template block<3, 1>(0, 3*j+2) < 0.0)
		++nnegatives;
	}
    }
    if (2*nnegatives > ncameras*nplanes)
    {
	for (size_t i = 0; i < ncameras; ++i)
	    P.template block<3, 1>(3*i, 3) *= -1.0;
	for (size_t j = 0; j < nplanes; ++j)
	    Q.template block<3, 3>(0, 3*j) *= -1.0;
    }
    
#ifdef _DEBUG
    for (size_t j = 0; j < nplanes; ++j)
    {
	std::cerr << " === " << j << "-th plane ===\n"
		  << "  Q30 = " << Q(3, 3*j) << ", Q31 = " << Q(3, 3*j+1)
		  << ", Q32 = " << Q(3, 3*j+2) << std::endl;
	const auto	Q3x2 = Q.template block<3, 2>(0, 3*j);
	std::cerr << " --- I(2x2) ---\n" <<  (Q3x2.transpose() * Q3x2).eval();
    }
    std::cerr << "*** End:   TU::projectiveToMetric() ***\n" << std::endl;
#endif    
}

/************************************************************************
*  class CameraCalibrator<T>						*
************************************************************************/
//! Zhangの方法により観測行列を1台のカメラの投影行列と複数の参照平面行列に分解する．
/*!
  \param W	参照平面数個の2次元射影変換行列を横に並べた観測行列
  \param P	カメラの3x4投影行列が返される
  \param Q	平面数個の参照平面行列を縦に並べた3x(3*平面数)行列が返される
*/
template <class T> void
CameraCalibrator<T>::zhangCalib(const matrix_type& W,
				matrix_type& P, matrix_type& Q)
{
    P.setZero(3, 4);
    Q = W;
    const auto	Kinv = swap_rows_and_cols(
			   matrix33_t(swap_rows_and_cols(
					  computeIAC(Q)).llt().matrixL()));
    P.template block<3, 3>(0, 0) = Kinv.inverse();
    Q = Kinv * Q;
}
    
//! 植芝の方法により観測行列を複数のカメラの投影行列と複数の参照平面行列に分解する．
/*!
  \param W		(カメラ数x参照平面数)の2次元射影変換行列から成る
			観測行列
  \param P		カメラ数個の投影行列を縦に並べた(3*カメラ数)x4行列が
			返される
  \param Q		平面数個の参照平面行列を縦に並べた4x(3*平面数)行列が
			返される
  \param scales		#computePlaneNormalizations で計算された
			各参照平面の2次元座標正規化変換のスケール
  \param commonCenters	全カメラの投影中心が共通ならばtrue,
			そうでなければfalse
*/
template <class T> void
CameraCalibrator<T>::ueshibaCalib(matrix_type& W,
				  matrix_type& P, matrix_type& Q,
				  const std::vector<T>& scales,
    				  bool commonCenters)
{
    rescaleHomographies(W);
    factorHomographies(W, P, Q, commonCenters);
    projectiveToMetric(P, Q, scales, commonCenters);
}

template class CameraCalibrator<float>;
template class CameraCalibrator<double>;
}
