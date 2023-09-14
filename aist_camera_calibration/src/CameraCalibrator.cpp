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
optimalEigenValue(const Matrix<T, 3, 3>& G)
{
    const Vector<T, 3>	a = G[1] ^ G[2], b = G[2] ^ G[0], c = G[0] ^ G[1];
    return -(a[1]*G[1][0] + a[2]*G[2][0] + b[0]*G[0][1] + b[2]*G[2][1] +
	     c[0]*G[0][2] + c[1]*G[1][2])
	   /(G[0][1]*G[0][1] + G[0][2]*G[0][2] +
	     G[1][0]*G[1][0] + G[1][2]*G[1][2] +
	     G[2][0]*G[2][0] + G[2][1]*G[2][1]);
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
template <class T> static Vector<T>
extpro(const Vector<T>& p, const Vector<T>& q)
{
    const size_t	d = p.size();
    Vector<T>		v(d * (d+1) / 2);
    int			n = 0;
    for (size_t i = 0; i < d; ++i)
    {
	v[n++] = p[i]*q[i];
	for (size_t j = i+1; j < d; ++j)
	    v[n++] = p[i]*q[j] + p[j]*q[i];
    }
    return v;
}

//! 射影空間において表現された参照平面行列から絶対円錐曲線の投影像(IAC)を計算する．
/*!
  \param Qt	平面数個の参照平面行列を縦に並べた(3*平面数)x[3|4]行列
  \return	IACを表す3x3対称行列
*/
template <class T> static Matrix<T, 3, 3>
computeIAC(const Matrix<T>& Qt)
{
    const size_t	nplanes = Qt.nrow() / 3;
    
  // Compute IAC(Image of Absolute Conic).
    Matrix<T>	A(6, 6);
    for (size_t j = 0; j < nplanes; ++j)
    {
	const Vector<T>	p = slice<3>(Qt[3*j], 0);
	const Vector<T>	q = slice<3>(Qt[3*j+1], 0);
	const Vector<T>	a = extpro(p, p) - extpro(q, q);
	const auto	b = extpro(p, q);
	A += (a % a + b % b);
    }
    
    Vector<T>		evalue;
    const auto		evector = eigen(A, evalue);
    Vector<T>		a;
    if (nplanes < 3)			// two reference planes.
	a = evector[5][1] * evector[4] - evector[4][1] * evector[5];
    else
	a = evector[5];
    Matrix<T, 3, 3>	omega({{a[0], a[1], a[2]},
			       {a[1], a[3], a[4]},
			       {a[2], a[4], a[5]}});

  // Make the lower-right element of omega.inv(), i.e. DIAC, becomes unity.
    omega *= (omega[0][0]*omega[1][1] - omega[0][1]*omega[1][0]) / det(omega);

    return omega;
}

//! 特異値分解ができるように観測行列のスケールを調整する．
/*!
  \param W	(カメラ数x参照平面数)の2次元射影変換行列から成る観測行列を
		与えるとそのスケールを調整されたものが返される．  
*/
template <class T> static void
rescaleHomographies(Matrix<T>& W)
{
#ifdef _DEBUG
    using namespace	std;
    
    cerr << "*** Begin: TU::rescaleHomographies() ***" << endl;
#endif
    const auto	ncameras = W.nrow()/3;
    const auto	nplanes  = W.ncol()/3;
    const auto	H00inv   = inverse(slice<3, 3>(W, 0, 0));
    for (size_t i = 1; i < ncameras; ++i)		// for each camera...
    {
      // Inter-image homography between camera 0 and i through plane 0.
	const auto	A = slice<3, 3>(W, 3*i, 0) * H00inv;

	for (size_t j = 1; j < nplanes; ++j)	// for each plane...
	{
	    auto	Hij = slice<3, 3>(W, 3*i, 3*j);
	    const Matrix<T, 3, 3>
			G   = slice<3, 3>(W, 0, 3*j) * inverse(Hij) * A;
	    const T	mu  = optimalEigenValue(G);
	    Hij *= mu;
#ifdef _DEBUG
	    const T	trG = trace(G), trAdjG = trace(adjoint(G));
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
  \param Qt		平面数個の参照平面行列を縦に並べた(3*平面数)x4行列が
			返される
  \param commonCenters	全カメラの投影中心が共通ならばtrue,
			そうでなければfalse
*/
template <class T> static void
factorHomographies(const Matrix<T>& W, Matrix<T>& P, Matrix<T>& Qt,
		   bool commonCenters)
{
    SVDecomposition<T>	svd(W);
#ifdef _DEBUG
    using namespace		std;
    
    cerr << "*** Begin: TU::factorHomographies() ***\n"
	 << " singular values: " << svd.diagonal()(0, 5);
#endif
    P.resize(W.nrow(), 4);
    Qt.resize(W.ncol(), 4);
    const size_t	rank = (commonCenters ? 3 : 4);
    for (size_t n = 0; n < rank; ++n)
    {
	for (size_t i = 0; i < P.nrow(); ++i)
	    P[i][n] = svd.Vt()[n][i];
	for (size_t j = 0; j < Qt.nrow(); ++j)
	    Qt[j][n] = svd[n] * svd.Ut()[n][j];
    }
#ifdef _DEBUG
    cerr << "*** End:   TU::factorHomographies() ***\n" << endl;
#endif
}

//! カメラの投影行列と参照平面行列を射影空間からユークリッド空間における表現に変換する．
/*!
  \param P		カメラ数個の投影行列を縦に並べた行列を与えると，
			それがユークリッド空間における表現に直されて返される
  \param Qt		平面数個の参照平面行列を縦に並べた行列を与えると，
			それがユークリッド空間における表現に直されて返される
  \param scales		#computePlaneNormalizations で計算された各参照平面の
			2次元座標正規化変換のスケール
  \param commonCenters	全カメラの投影中心が共通ならばtrue, そうでなければfalse
*/ 
template <class T> static void
projectiveToMetric(Matrix<T>& P, Matrix<T>& Qt, const Vector<T>& scales,
		   bool commonCenters)
{
    using vector4_type	= Vector<T, 4>;
    using matrix44_type	= Matrix<T, 4, 4>;
#ifdef _DEBUG
    using namespace	std;

    cerr << "*** Begin: TU::projectiveToMetric() ***" << endl;
#endif
    const auto		nplanes = Qt.nrow() / 3;
    
  // Fix WC(World Coordinates) to 0-th camera.
    SVDecomposition<T>	svd(slice<3, 4>(P, 0, 0));
    matrix44_type	Sinv;
    slice<3, 4>(Sinv, 0, 0) = slice<3, 4>(P, 0, 0);
    Sinv[3] = svd.Ut()[3];
#ifdef _DEBUG
    cerr << "  det(Sinv) = " << det(Sinv) << endl;
#endif    
    P  = evaluate(P  * inverse(Sinv));
    Qt = evaluate(Qt * transpose(Sinv));

  // Compute IAC(Image of Absolute Conic) of 0-th camera.
    const auto	omega = computeIAC(Qt);
    const auto	K1inv = cholesky(omega);

    if (commonCenters)
    {
	P( 0, P.nrow(),  0, 3) = evaluate(P( 0, P.nrow(),  0, 3) *
					  inverse(K1inv));
	Qt(0, Qt.nrow(), 0, 3) = evaluate(Qt(0, Qt.nrow(), 0, 3) *
					  transpose(K1inv));
    }
    else
    {
      // Compute PI(Plane at Inifinity).
	Vector<T>	b(3*nplanes), beta(nplanes);
	for (size_t j = 0; j < nplanes; ++j)
	{
	    const auto	Qt2x3 = slice<2, 3>(Qt, 3*j, 0);
	    beta[j] = sqrt((Qt2x3[0]*omega*Qt2x3[0] +
			    Qt2x3[1]*omega*Qt2x3[1])/2.0);
#ifdef _DEBUG
	    cerr << "  " << j << "-th plane: beta = " << beta[j] << endl;
#endif
	    b[3*j  ] = 0.0;
	    b[3*j+1] = 0.0;
	    b[3*j+2] = beta[j] * scales[j];
	}
	vector4_type	h = b * Qt;		// PI(Plane at Infinity)
	solve(transpose(Qt) * Qt, h);

      // Compute transformation from projective to Euclidean coordinates.
	matrix44_type	TT;
	slice<3, 3>(TT, 0, 0) = K1inv;
	TT[3] = h;
	P  = evaluate(P  * inverse(TT));
	Qt = evaluate(Qt * transpose(TT));
	for (size_t j = 0; j < nplanes; ++j)
	    slice<3, 4>(Qt, 3*j, 0) /= beta[j];
    }
    
  // Ensure the left 3x3 part of camera matrices to have positive determinant.
    const size_t	ncameras = P.nrow() / 3;
    for (size_t i = 0; i < ncameras; ++i)
	if (det(slice<3, 3>(P, 3*i, 0)) < 0.0)
	    slice<3, 4>(P, 3*i, 0) *= -1.0;

  // Ensure that the plane origins are in front of the cameras.
    size_t	nnegatives = 0;
    for (size_t i = 0; i < ncameras; ++i)
    {
	const auto	r3 = slice<3>(P[3*i+2], 0);
	
	for (size_t j = 0; j < nplanes; ++j)
	{
	    if (r3 * slice<3>(Qt[3*j+2], 0) < 0.0)
		++nnegatives;
	}
    }
    if (2*nnegatives > ncameras*nplanes)
    {
	for (size_t i = 0; i < ncameras; ++i)
	    slice<3, 1>(P, 3*i, 3) *= -1.0;
	for (size_t j = 0; j < nplanes; ++j)
	    slice<3, 3>(Qt, 3*j, 0) *= -1.0;
    }
    
#ifdef _DEBUG
    for (size_t j = 0; j < nplanes; ++j)
    {
	cerr << " === " << j << "-th plane ===\n"
	     << "  Q30 = " << Qt[3*j  ][3] << ", Q31 = " << Qt[3*j+1][3]
	     << ", Q32 = " << Qt[3*j+2][3] << endl;
	const auto	Qt2x3 = slice<2, 3>(Qt, 3*j, 0);
	cerr << " --- I(2x2) ---\n" <<  evaluate(Qt2x3 * transpose(Qt2x3));
    }
    cerr << "*** End:   TU::projectiveToMetric() ***\n" << endl;
#endif    
}

/************************************************************************
*  class CameraCalibrator<T>						*
************************************************************************/
//! Zhangの方法により観測行列を1台のカメラの投影行列と複数の参照平面行列に分解する．
/*!
  \param W	参照平面数個の2次元射影変換行列を横に並べた観測行列
  \param P	カメラの3x4投影行列が返される
  \param Qt	平面数個の参照平面行列を縦に並べた(3*平面数)x3行列が返される
*/
template <class T> void
CameraCalibrator<T>::zhangCalib(const matrix_type& W,
				matrix_type& P, matrix_type& Qt)
{
    P.resize(3, 4);
    Qt = transpose(W);
    const matrix33_type	Kinv = cholesky(computeIAC(Qt));
    slice<3, 3>(P, 0, 0) = inverse(Kinv);
    Qt = evaluate(Qt * transpose(Kinv));
}
    
//! 植芝の方法により観測行列を複数のカメラの投影行列と複数の参照平面行列に分解する．
/*!
  \param W		(カメラ数x参照平面数)の2次元射影変換行列から成る
			観測行列
  \param P		カメラ数個の投影行列を縦に並べた(3*カメラ数)x4行列が
			返される
  \param Qt		平面数個の参照平面行列を縦に並べた(3*平面数)x4行列が
			返される
  \param scales		#computePlaneNormalizations で計算された
			各参照平面の2次元座標正規化変換のスケール
  \param commonCenters	全カメラの投影中心が共通ならばtrue,
			そうでなければfalse
*/
template <class T> void
CameraCalibrator<T>::ueshibaCalib(matrix_type& W,
				  matrix_type& P, matrix_type& Qt,
				  const vector_type& scales,
    				  bool commonCenters)
{
    rescaleHomographies(W);
    factorHomographies(W, P, Qt, commonCenters);
    projectiveToMetric(P, Qt, scales, commonCenters);
}

template class CameraCalibrator<float>;
template class CameraCalibrator<double>;
}
