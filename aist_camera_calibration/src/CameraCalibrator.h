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
#ifndef __CAMERACALIBRATOR_H
#define __CAMERACALIBRATOR_H

/*!
  \file		CameraCalibrator.h
  \brief	クラス#TU::CameraCalibratorの定義と実装
*/
#include "TU/Geometry++.h"
#include "TU/BlockDiagonalMatrix++.h"

namespace TU
{
/************************************************************************
*  class ReferencePlane<T>						*
************************************************************************/
//! キャリブレーションパターンを描いた参照平面を表すクラス
template <class T>
class ReferencePlane
{
  public:
    typedef T						element_type;
    typedef Point2<element_type>			point2_type;
    typedef Point3<element_type>			point3_type;
    typedef Vector<element_type>			vector_type;
    typedef Matrix<element_type>			matrix_type;
    typedef Matrix<element_type, 3, 3>			matrix33_type;
    typedef PlaneP<element_type>			plane_type;
    
  public:
  //! 参照平面を生成する．
    ReferencePlane()	:_d(), _Rt()					{}
    
    void		initialize(const matrix33_type& Qt)		;
    const point3_type&	d()					const	;
    const matrix33_type&
			Rt()					const	;
    matrix33_type	Qt()					const	;
    plane_type		h()					const	;
    point3_type		operator ()(const point2_type& x)	const	;
    matrix_type		derivative(const point2_type& x)	const	;
    void		update(const vector_type& dq)			;
    
  private:
    point3_type		_d;		//!< 参照平面の位置
    matrix33_type	_Rt;		//!< 参照平面の姿勢
};

//! 参照平面の位置と姿勢を設定する．
/*!
  \param Qt	位置と姿勢を表す3x3行列．1行目／2行目は，平面上に設定された座標
		系の横軸／縦軸の向き，3行目は座標系原点の位置(いずれもワールド
		座標系から見たもの)．すなわち\f$\TUtvec{Q}{} =
		[\TUvec{i}{}, \TUvec{j}{}, \TUvec{d}{}]^\top\f$
*/
template <class T> inline void
ReferencePlane<T>::initialize(const matrix33_type& Qt)
{
  // Optimal rotation matrix representing the orientation of the plane.
    SVDecomposition<element_type>	svd(slice<2, 3>(Qt, 0, 0));
    slice<2, 3>(_Rt, 0, 0) = transpose(svd.Vt()) * slice<2, 3>(svd.Ut(), 0, 0);
    _Rt[2] = _Rt[0] ^ _Rt[1];
    
  // Location of the plane.
    _d = Qt[2] / sqrt(square(svd.diagonal()) / 2);

  /*element_type	scale = Qt[0].length();
    (_Rt[0] = Qt[0]) /= scale;
    (_Rt[2] = Qt[0] ^ Qt[1]).normalize();
    _Rt[1] = _Rt[2] ^ _Rt[0];

    (_d = Qt[2]) /= scale;*/
}

//! 参照平面の位置を返す．
/*!
  \return	参照平面の位置
*/
template <class T> inline const typename ReferencePlane<T>::point3_type&
ReferencePlane<T>::d() const
{
    return _d;
}
    
//! 参照平面の姿勢を返す．
/*!
  \return	参照平面の姿勢
*/
template <class T> inline const typename ReferencePlane<T>::matrix33_type&
ReferencePlane<T>::Rt() const
{
    return _Rt;
}
    
//! 参照平面の位置と姿勢を返す．
/*!
  \return	位置と姿勢を表す3x3行列．1行目／2行目は，平面上に設定された
		座標系の横軸／縦軸の向き，3行目は座標系原点の位置(いずれも
		ワールド座標系から見たもの)．すなわち\f$\TUtvec{Q}{} =
		[\TUvec{i}{}, \TUvec{j}{}, \TUvec{d}{}]^\top\f$
*/
template <class T> inline typename ReferencePlane<T>::matrix33_type
ReferencePlane<T>::Qt() const
{
    matrix33_type	Qt;
    Qt[0] = _Rt[0];
    Qt[1] = _Rt[1];
    Qt[2] = _d;

    return Qt;
}

//! 参照平面の同次座標を返す．
/*!
  \return	参照平面を表す4次元ベクトル．すなわち\f$\TUvec{h}{} =
		[\TUtvec{k}{}, -\TUtvec{k}{}\TUvec{d}{}]^\top\f$
*/
template <class T> inline typename ReferencePlane<T>::plane_type
ReferencePlane<T>::h() const
{
    plane_type	h;
    h(0, 3) = _Rt[2];
    h[3]    = -_Rt[2] * _d;

    return h;
}

//! 参照平面上の点の2次元座標をワールド座標系から見た3次元座標に変換する．
/*!
  \param x	点の2次元座標
  \return	xの3次元座標，すなわち
		\f$\TUvec{X}{} = x\TUvec{i}{} + y\TUvec{j}{} + \TUvec{d}{}\f$
*/
template <class T> inline typename ReferencePlane<T>::point3_type
ReferencePlane<T>::operator ()(const point2_type& x) const
{
    return _d + x[0] * _Rt[0] + x[1] * _Rt[1];
}

//! 平面パラメータに関する参照平面上の点の3次元座標の1階微分を求める．
/*!
  \param x	点の2次元座標
  \return	3x6ヤコビ行列，すなわち
		\f$
		\TUbeginarray{cc}
		\TUdisppartial{\TUvec{X}{}}{\TUvec{d}{}} &
		\TUdisppartial{\TUvec{X}{}}{\TUvec{\theta}{}}
		\TUendarray = 
		\TUbeginarray{cc}
		\TUvec{I}{3\times 3} & x\TUskew{i}{} + y\TUskew{j}{}
		\TUendarray
		\f$
*/
template <class T> inline typename ReferencePlane<T>::matrix_type
ReferencePlane<T>::derivative(const point2_type& x) const
{
    matrix_type	J(3, 6);
    slice<3, 3>(J, 0, 0) = diag(element_type(1), 3);
    slice<3, 3>(J, 0, 3) = x[0] * skew(_Rt[0]) + x[1] * skew(_Rt[1]);

    return J;
}

//! 平面パラメータを指定された量だけ更新する．
/*!
  \param dq	更新量を表す6次元ベクトル
*/
template <class T> inline void
ReferencePlane<T>::update(const vector_type& dq)
{
    _d -= slice<3>(dq, 0);
    _Rt = evaluate(_Rt * rotation(slice<3>(dq, 3)));
}

//! 出力ストリームに平面パラメータを3x3行列の形式で書き出す(ASCII)．
/*!
  \param out	出力ストリーム
  \return	outで指定した出力ストリーム
*/
template <class T> inline std::ostream&
operator <<(std::ostream& out, const ReferencePlane<T>& plane)
{
    using namespace	std;
    typedef T		element_type;
    
    const element_type	DEG = element_type(180) / element_type(M_PI);
    cerr << "Position:       ";
    out << plane.d();
    cerr << "Rotation(deg.): ";
    out << evaluate(DEG * rotation_axis(plane.Rt()));

    return out;
}
    
/************************************************************************
*  class CameraCalibrator<T>						*
************************************************************************/
//! 参照物体を利用してカメラキャリブレーションを行うクラス
template <class T>
class CameraCalibrator
{
  public:
    typedef T						element_type;
    typedef ReferencePlane<element_type>		plane_type;
    typedef Vector<element_type>			vector_type;
    typedef Matrix<element_type>			matrix_type;
    typedef Point2<element_type>			point2_type;
    typedef Point3<element_type>			point3_type;
    typedef Matrix<element_type, 3, 3>			matrix33_type;

  private:
    typedef Normalize<element_type, 2>			normalize_type;
    
  private:
  //! #volumeCalibにおいて非線形最適化を行う場合にその二乗を最小化すべき再投影誤差関数
  /*!
    \param Iter	課される要件は#volumeCalibと同じ
    \param Cam	課される要件は#volumeCalibと同じ
  */
    template <class Iter, class Cam>
    class VolumeCost
    {
      public:
	typedef CameraCalibrator::element_type		element_type;
	
      public:
	VolumeCost(Iter begin, Iter end)				;

	vector_type	operator ()(const Cam& camera)		const	;
	matrix_type	derivative(const Cam& camera)		const	;
	void		update(Cam& camera,
			       const vector_type& dc)		const	;
	
	element_type	reprojectionError(const Cam& camera)	const	;
	matrix_type	standardDeviations(const Cam& camera,
					   const matrix_type& covariance)
								const	;
	
      private:
	const Iter	_begin;		//!< 点対列の先頭を示す反復子
	const Iter	_end;		//!< 点対列の末尾の次を示す反復子
	const size_t	_npoints;	//!< [_begin, _end)間に含まれる点対の個数
    };
    
  //! #planeCalibにおいて非線形最適化を行う場合にその二乗を最小化すべき再投影誤差関数
  /*!
    \param Iter	課される要件は#planeCalibと同じ
    \param Cam	課される要件は#planeCalibと同じ
  */
    template <class Iter, class Cam>
    class PlaneCost
    {
      public:
	typedef CameraCalibrator::element_type		element_type;
	typedef BlockDiagonalMatrix<element_type>	derivative_type;
      //! 特定の参照平面について，全カメラに渡る参照点とその投影像の対データ
	typedef typename Iter::value_type		CorresListArray;
      //! 特定の参照平面とカメラについて，それらの間の参照点とその投影像の対データ
	typedef typename CorresListArray::value_type	CorresList;
    
	PlaneCost(Iter begin, Iter end, bool commonCenters)		;

      //! 全カメラの総自由度数を返す．
	size_t			adim()		const	{return _adim;}
      //! 各カメラの自由度を返す．
	const Array<size_t>&	adims()		const	{return _adims;}
      //! 1枚の参照平面の自由度を返す．
	size_t			bdim()		const	{return 6;}
	
	vector_type	operator ()(const Array<Cam>& cameras,
				    const plane_type& plane, int j)
								const	;
	derivative_type	derivativeA(const Array<Cam>& cameras,
				  const plane_type& plane, int j)
								const	;
	matrix_type	derivativeB(const Array<Cam>& cameras,
				  const plane_type& plane, int j)
								const	;
	void		updateA(Array<Cam>& cameras,
				const vector_type& dcameras)	const	;
	void		updateB(plane_type& plane,
				const vector_type& dplane)	const	;
    
	element_type	reprojectionError(const Array<Cam>& cameras,
					  const Array<plane_type>& planes)
								const	;
	matrix_type	standardDeviations(const Array<Cam>& cameras,
					   const matrix_type& covariance)
								const	;
	
      private:
      //! カメラの台数を返す．
	size_t		ncameras()		const	{return _adims.size();}
	const CorresListArray&
			corresListArray(int j)	const	;
    
	const Iter	_begin;		//!< 参照平面データの先頭を示す反復子
	const Iter	_end;		//!< 参照平面データの末尾の次を示す反復子
	size_t		_adim;		//!< 全カメラの総自由度数
	Array<size_t>	_adims;		//!< 各カメラの自由度
	Array<size_t>	_npoints;	//!< 各参照平面に関わる点対数
    };

  public:
  //! カメラキャリブレーション器を生成する．
    CameraCalibrator()
	:_reprojectionError(0.0), _standardDeviations(0, 0)		{}

  //! キャリブレーションによって求められた再投影誤差(単位：pixel)を返す．
  /*!
    \return	再投影誤差
  */
    element_type
		reprojectionError()	  const	{return _reprojectionError;}

  //! キャリブレーションによって求められたカメラパラメータの標準偏差を返す．
  /*!
    \return	各行にカメラ毎の推定パラメータの標準偏差を収めた
		(カメラ数x内部・外部パラメータ数)行列
  */
    const matrix_type&
		standardDeviations()	  const	{return _standardDeviations;}

  //! キャリブレーションによって求められたカメラパラメータの標準偏差を返す．
  /*!
    \param i	カメラを指定するindex
    \return	各行にカメラ毎の推定パラメータの標準偏差を収めた
		内部・外部パラメータ数次元のベクトル
  */
    const vector_type
		standardDeviations(int i) const	{return _standardDeviations[i];}
    
    template <class Iter, class Cam> void
		volumeCalib(Iter begin, Iter end,
			    Cam& camera, bool refine)			;
    template <class Iter, class Cam> Array<plane_type>
		planeCalib(Iter begin, Iter end, Array<Cam>& cameras,
			   bool commonCenters, bool refine)		;

  private:
    template <class Iter> static Array<normalize_type>
		computeCameraNormalizations(Iter begin, Iter end)	;
    template <class Iter> static Array<normalize_type>
		computePlaneNormalizations(Iter begin, Iter end)	;
    template <class Iter> static matrix_type
		computeHomographies(Iter begin, Iter end)		;
    static void	zhangCalib(const matrix_type& W,
			   matrix_type& P, matrix_type& Qt)		;
    static void	ueshibaCalib(matrix_type& W,
			     matrix_type& P, matrix_type& Qt,
			     const vector_type& scales,
			     bool commonCenters)			;

    element_type	_reprojectionError;
    matrix_type		_standardDeviations;
};

//! 3次元参照物体を利用してカメラキャリブレーションを行う．
/*!
  テンプレートパラメータIterは以下の条件を満たすこと：
  -# 参照物体上の参照点の3次元座標とその投影像の2次元座標の組
	std::pair<Ref, Proj>
     を指す前進反復子(forward iterator)である．

  テンプレートパラメータCamは以下の条件を満たすこと：
  -# Camera型もしくはCameraWithDistortion型

  \param begin		点対列の先頭を示す反復子
  \param end		点対列の末尾の次を示す反復子
  \param camera		カメラパラメータの推定値が返される
  \param refine		非線形最適化を行って解の精度を高めるならばtrue,
			そうでなければfalse
*/
template <class T> template <class Iter, class Cam> void
CameraCalibrator<T>::volumeCalib(Iter begin, Iter end, Cam& camera, bool refine)
{
    Projectivity23<T>		map(begin, end, false);
    camera.setProjection(map);

    VolumeCost<Iter, Cam>	cost(begin, end);
    if (refine)
    {
	NullConstraint<element_type>	g;
	const matrix_type&		S = minimizeSquare(cost, g, camera);
	_standardDeviations = cost.standardDeviations(camera, S);
    }
    else
	_standardDeviations.resize(0, 0);
    _reprojectionError = cost.reprojectionError(camera);
}

//! 2次元参照平面を利用してカメラキャリブレーションを行う．
/*!
  カメラが1台の場合はZhangの方法，2台以上の場合は植芝の方法でキャリブレーションを行う．

  テンプレートパラメータIterは以下の条件を満たすこと：
  -# TU::Array<Container>
     を指す前進反復子(forward iterator)である．
  -# Array<Container>は1枚の参照平面と複数のカメラ間の対応点データを収めた配列で，
     そのサイズはカメラの台数に等しい．
  -# Containerは1枚の参照平面と1台のカメラ間の対応点データを収めたコンテナで，
     参照点とその投影像の2次元座標の組
	std::pair<Ref, Proj>
     を要素とし，前進反復子(foraward iterator)をサポートする．

  テンプレートパラメータCamは以下の条件を満たすこと：
  -# Camera型もしくはCameraWithDistortion型

  \param begin		最初の参照平面から得られたデータを示す反復子
  \param end		最後の参照平面から得られたデータの次を示す反復子
  \param cameras	カメラパラメータの推定値が返される
  \param commonCenters	全カメラの投影中心が一致しているならばtrue,
			そうでなければfalse
  \param refine		非線形最適化を行って解の精度を高めるならばtrue,
			そうでなければfalse
  \return		推定された参照平面
*/
template <class T> template <class Iter, class Cam>
Array<typename CameraCalibrator<T>::plane_type>
CameraCalibrator<T>::planeCalib(Iter begin, Iter end, Array<Cam>& cameras,
				bool commonCenters, bool refine)
{
    using namespace	std;
    
#ifdef _DEBUG
    cerr << "*** Begin: TU::CameraCalibrator<T>::planeCalib() ***" << endl;
#endif
  // Compute homography matrices for each camera-plane pair.
    auto	W = computeHomographies(begin, end);
    const auto	ncameras = W.nrow() / 3;
    const auto	nplanes  = W.ncol() / 3;

  // Normalize the homography matrices.
    Array<normalize_type >
	cameraNorms = computeCameraNormalizations(begin, end);
    Array<normalize_type >
	planeNorms  = computePlaneNormalizations(begin, end);
    for (size_t i = 0; i < ncameras; ++i)
	for (size_t j = 0; j < nplanes; ++j)
	    slice<3, 3>(W, 3*i, 3*j)
		= evaluate(cameraNorms[i].T() *
			   slice<3, 3>(W, 3*i, 3*j) * planeNorms[j].Tinv());

  // Factor the homography matrices into cameras and planes.
    matrix_type	P, Qt;				// cameras and planes
    if (ncameras <= 1)
	zhangCalib(W, P, Qt);			// Zhang's algorithm
    else
    {
      // Extract scale factors for each plane.
	vector_type	scales(nplanes);
	for (size_t j = 0; j < nplanes; ++j)
	    scales[j] = planeNorms[j].T()[0][0];

	ueshibaCalib(W, P, Qt, scales, commonCenters);	// Ueshiba's algorithm
    }
    
  // Unnormalize computed camera and plane parameters.
    for (size_t i = 0; i < ncameras; ++i)
	slice<3, 4>(P, 3*i, 0) = evaluate(cameraNorms[i].Tinv() *
					  slice<3, 4>(P, 3*i, 0));
    for (size_t j = 0; j < nplanes; ++j)
	Qt(3*j, 3, 0, Qt.ncol())
	    = evaluate(planeNorms[j].Tt() * Qt(3*j, 3, 0, Qt.ncol()));

  // Set camera parameters.
    cameras.resize(ncameras);
    for (size_t i = 0; i < cameras.size(); ++i)
	cameras[i].setProjection(slice<3, 4>(P, 3*i, 0));

  // Set plane parameters.
    Array<plane_type>	planes(nplanes);
    for (size_t j = 0; j < planes.size(); ++j)
	planes[j].initialize(slice<3, 3>(Qt, 3*j, 0));

#ifdef _DEBUG
    cerr << "--- Camera parameters linearly estimated..." << endl;
#endif
  // Do refinement if required.
    PlaneCost<Iter, Cam>	cost(begin, end, commonCenters);
    if (refine)
    {
	NullConstraint<element_type>	g;
	const matrix_type&
	    S = minimizeSquareSparse(cost, g, cameras,
				     planes.begin(), planes.end(), 1000);
	_standardDeviations = cost.standardDeviations(cameras, S);
#ifdef _DEBUG
	cerr << "--- Camera parameters refined..." << endl;
#endif
    }
    else
	_standardDeviations.resize(0, 0);
    _reprojectionError  = cost.reprojectionError(cameras, planes);
#ifdef _DEBUG
    cerr << "\n*** End:   TU::CameraCalibrator<T>::planeCalib() ***\n"
	 << endl;
#endif
    return planes;
}

//! 2次元射影変換行列に対して計算安定化のための投影像正規化変換を計算する．
/*!
  参照点の投影像の2次元座標のための正規化変換(各カメラ毎に異なる)
  \f$\TUvec{S}{1}, \TUvec{S}{2},\ldots, \TUvec{S}{I}\f$が計算される．
  このようにして求められた正規化変換は，参照点の2次元座標のための正規化変換
  \f$\TUvec{T}{1}, \TUvec{T}{2},\ldots, \TUvec{T}{J}\f$と併せて
  参照平面jからカメラiへの射影変換\f$\TUvec{H}{i}^j\f$を
  \f$\TUbar{H}{i}^j = \TUvec{S}{i}\TUvec{H}{i}^j\TUinv{T}{j}\f$
  と変換するために用いられる．
  
  テンプレートパラメータIterに課される要件は#planeCalibと同じ．
  \param begin		最初の参照平面から得られたデータを示す反復子
  \param end		最後の参照平面から得られたデータの次を示す反復子
  \return		(カメラ数)個の正規化変換
*/
template <class T>
template <class Iter> Array<typename CameraCalibrator<T>::normalize_type>
CameraCalibrator<T>::computeCameraNormalizations(Iter begin, Iter end)
{
    Array<normalize_type>	norms(begin != end ? begin->size() : 0);

    for (auto iter = begin; iter != end; ++iter)	// 各参照平面について
	for (size_t i = 0; i < iter->size(); ++i)	// 各カメラについて
	    norms[i].insert(make_second_iterator((*iter)[i].begin()),
			    make_second_iterator((*iter)[i].end()));

    return norms;
}

//! 2次元射影変換行列に対して計算安定化のための参照点正規化変換を計算する．
/*!
  参照点の2次元座標のための正規化変換(各参照平面毎に異なる)
  \f$\TUvec{T}{1}, \TUvec{T}{2},\ldots, \TUvec{T}{J}\f$が計算される．
  このようにして求められた正規化変換は，投影像の2次元座標のための正規化変換
  \f$\TUvec{S}{1}, \TUvec{S}{2},\ldots, \TUvec{S}{I}\f$と併せて
  参照平面jからカメラiへの射影変換\f$\TUvec{H}{i}^j\f$を
  \f$\TUbar{H}{i}^j = \TUvec{S}{i}\TUvec{H}{i}^j\TUinv{T}{j}\f$
  と変換するために用いられる．
  
  テンプレートパラメータIterに課される要件は#planeCalibと同じ．
  \param begin		最初の参照平面から得られたデータを示す反復子
  \param end		最後の参照平面から得られたデータの次を示す反復子
  \return		(参照点数)個の正規化変換
*/
template <class T>
template <class Iter> Array<typename CameraCalibrator<T>::normalize_type>
CameraCalibrator<T>::computePlaneNormalizations(Iter begin, Iter end)
{
    Array<normalize_type>	norms(std::distance(begin, end));

    size_t	j = 0;
    for (auto iter = begin; iter != end; ++iter)	// 各参照平面について
    {
	for (size_t i = 0; i < iter->size(); ++i)	// 各カメラについて
	    norms[j].insert(make_first_iterator((*iter)[i].begin()),
			    make_first_iterator((*iter)[i].end()));
	++j;
    }

    return norms;
}

//! 参照平面とカメラの画像平面間の2次元射影変換行列を計算する．
/*!
  観測行列
  \f[
  \TUvec{W}{} =
  \TUbeginarray{ccc}
  \TUvec{H}{1}^1 & \cdots & \TUvec{H}{1}^J \\ \vdots & & \vdots \\
  \TUvec{H}{I}^1 & \cdots & \TUvec{H}{I}^J \\
  \TUendarray
  \f]
  が計算される．

  テンプレートパラメータIterに課される要件は#planeCalibと同じ．
  \param begin		最初の参照平面から得られたデータを示す反復子
  \param end		最後の参照平面から得られたデータの次を示す反復子
  \return		(カメラ数x参照平面数)の2次元射影変換行列から成る観測行列
*/
template <class T>
template <class Iter> typename CameraCalibrator<T>::matrix_type
CameraCalibrator<T>::computeHomographies(Iter begin, Iter end)
{
    using namespace	std;

    const auto	nplanes  = std::distance(begin, end);
    if (nplanes < 2)
	throw runtime_error("Need two or more planes!!");
    const auto	ncameras = begin->size();
#ifdef _DEBUG
    cerr << "*** Begin: TU::CameraCalibrator<T>::computeHomographies() ***\n "
	 << ncameras << " cameras observing " << nplanes << " planes."
	 << endl;
#endif
    matrix_type	W(3*ncameras, 3*nplanes);
    int		j = 0;
    for (auto iter = begin; iter != end; ++iter)
    {
	if (iter->size() != ncameras)
	    throw runtime_error("All the planes must be observed by the same number of cameras!!");
	
	for (size_t i = 0; i < ncameras; ++i)
	{
	    Projectivity22<T>	H((*iter)[i].begin(), (*iter)[i].end(), true);
	    slice<3, 3>(W, 3*i, 3*j) = H;
#ifdef _DEBUG
	    cerr << "--- H" << i << j << " (RMS-err: "
		 << H.rmsError((*iter)[i].begin(), (*iter)[i].end())
		 << ") ---\n"
		 << slice<3, 3>(W, 3*i, 3*j);
#endif
	}
	++j;
    }
#ifdef _DEBUG
    cerr << "*** End:   TU::CameraCalibrator<T>::computeHomographies() ***\n"
	 << endl;
#endif
    return W;
}

/************************************************************************
*  class CameraCalibrator<T>::VolumeCost<Iter, Cam>			*
************************************************************************/
//! 参照点の3次元座標とその投影像の2次元座標の対を与えて再投影誤差関数を初期化する．
/*!
  \param begin		点対列の先頭を示す反復子
  \param end		点対列の末尾の次を示す反復子
*/
template <class T> template <class Iter, class Cam> inline
CameraCalibrator<T>::VolumeCost<Iter, Cam>::VolumeCost(Iter begin, Iter end)
    :_begin(begin), _end(end), _npoints(std::distance(_begin, _end))
{
}
    
//! 与えられたカメラパラメータ値における再投影誤差を計算する．
/*!
  #VolumeCost()で与えた全ての点対について再投影誤差が計算される．
  \param camera		カメラパラメータ
  \return		再投影誤差を収めた(2*点対数)次元ベクトル
*/
template <class T>
template <class Iter, class Cam> typename CameraCalibrator<T>::vector_type
CameraCalibrator<T>::VolumeCost<Iter, Cam>::operator ()(const Cam& camera) const
{
    vector_type	val(2*_npoints);
    size_t	n = 0;
    for (Iter iter = _begin; iter != _end; ++iter)
    {
	val(n, 2) = camera(iter->first) - iter->second;
	n += 2;
    }

    return val;
}
    
//! 与えられたカメラパラメータ値においてカメラパラメータに関する再投影誤差の1階微分を計算する．
/*!
  #VolumeCost()で与えた全ての点対についてヤコビ行列が計算される．
  \param camera		カメラパラメータ
  \return		(2*点対数)x(6+内部パラメータ数)ヤコビ行列
*/
template <class T>
template <class Iter, class Cam> typename CameraCalibrator<T>::matrix_type
CameraCalibrator<T>::VolumeCost<Iter, Cam>::derivative(const Cam& camera) const
{
    matrix_type	J(2*_npoints, 6 + Cam::dofIntrinsic());
    size_t	n = 0;
    for (Iter iter = _begin; iter != _end; ++iter)
    {
	typename Cam::matrix_type	JJ;
	camera(iter->first, &JJ);
	J(n, 2, 0, J.ncol()) = JJ;
	n += 2;
    }

    return J;
}

//! カメラパラメータを更新する．
/*!
  \param camera		更新されるカメラパラメータ
  \param dc		更新量
*/
template <class T> template <class Iter, class Cam> inline void
CameraCalibrator<T>::VolumeCost<Iter, Cam>::update(Cam& camera,
						   const vector_type& dc) const
{
    camera.update(dc);
}
	
//! 再投影誤差の全点対に渡る平均値を返す．
/*!
  \param camera		カメラパラメータ
  \return		再投影誤差の平均値
*/
template <class T> template <class Iter, class Cam>
inline T
CameraCalibrator<T>::VolumeCost<Iter, Cam>::reprojectionError(
    const Cam& camera) const
{
    return sqrt(operator ()(camera).square() / _npoints);
}
    
//! カメラパラメータの共分散行列からその標準偏差を求める．
/*!
  \param camera		カメラパラメータ
  \param covariance	カメラパラメータの共分散行列
  \return		標準偏差を収めた1x(6+内部パラメータ数)行列
*/
template <class T> template <class Iter, class Cam>
inline typename CameraCalibrator<T>::matrix_type
CameraCalibrator<T>::VolumeCost<Iter, Cam>::standardDeviations(
    const Cam& camera, const matrix_type& covariance) const
{
    matrix_type	stddevs(1, 6 + Cam::dofIntrinsic());
    for (size_t n = 0; n < stddevs.size(); ++n)
	stddevs[0][n] = sqrt(covariance[n][n] / _npoints);
    if (stddevs.ncol() > 9)
    {
	stddevs[0][9]  /= camera.k();
	stddevs[0][10] /= camera.k();
    }
    
    return stddevs;
}
    
/************************************************************************
*  class CameraCalibrator<T>::PlaneCost<Iter, Cam>			*
************************************************************************/
//! 複数の参照平面について，その上の参照点と投影像の2次元座標対を与えて再投影誤差関数を初期化する．
/*!
  \param begin		参照平面データ列の先頭を示す反復子
  \param end		参照平面データ列の末尾の次を示す反復子
  \param commonCenters	全カメラの投影中心が一致しているならばtrue,
			そうでなければfalse
*/
template <class T> template <class Iter, class Cam> 
CameraCalibrator<T>::PlaneCost<Iter, Cam>::PlaneCost(Iter begin, Iter end,
						     bool commonCenters)
    :_begin(begin), _end(end), _adim(0),
     _adims(_begin != _end ? _begin->size() : 0),
     _npoints(std::distance(_begin, _end))
{
  // Compute DOF values for each camera.
    _adims[0] = Cam::dofIntrinsic();
    _adim += _adims[0];
    for (size_t i = 1; i < _adims.size(); ++i)		// for each camera...
    {
	_adims[i] = (commonCenters ? 3 : 6) + _adims[0];
	_adim += _adims[i];
    }

  // Compute the total number of points observed by all the cameras
  // for each plane.
    size_t	j = 0;
    for (auto iter = _begin; iter != _end; ++iter)	// for each plane...
    {
	_npoints[j] = 0;
	for (size_t i = 0; i < iter->size(); ++i)	// for each camera...
	    _npoints[j] += (*iter)[i].size();
	++j;
    }
}

//! 指定された参照平面について，与えられたカメラ／平面パラメータ値における再投影誤差を計算する．
/*!
  指定した参照平面に関わる#PlaneCost()で与えた全ての点対に対して
  再投影誤差が計算される．
  \param cameras	カメラパラメータを収めた配列
  \param plane		参照平面
  \param j		参照平面を指定するindex
  \return		再投影誤差を収めた(2*参照平面jに関わる点対数)
			次元ベクトル
*/
template <class T>
template <class Iter, class Cam> typename CameraCalibrator<T>::vector_type
CameraCalibrator<T>::PlaneCost<Iter, Cam>::operator ()(
    const Array<Cam>& cameras, const plane_type& plane, int j) const
{
    const auto&	data = corresListArray(j);
    vector_type	val(2 * _npoints[j]);
    for (size_t k = 0, i = 0; i < ncameras(); ++i)
	for (auto iter = data[i].begin(); iter != data[i].end(); ++iter)
	{
	    val(k, 2) = cameras[i](plane(iter->first)) - iter->second;
	    k += 2;
	}

    return val;
}

//! 指定された参照平面について，カメラパラメータに関する再投影誤差の1階微分を計算する．
/*!
  指定した参照平面に関わる#PlaneCost()で与えた全ての点対に対して
  ヤコビ行列が計算される．
  \param cameras	カメラパラメータを収めた配列
  \param plane		参照平面
  \param j		参照平面を指定するindex
  \return		カメラ台数個の小ヤコビ行列から成るブロック対角行列．
			各小ヤコビ行列のサイズは，
			1台目のカメラが(2*点対数)x(内部パラメータ数)，
			2台目以降のカメラが(2*点対数)x(6+内部パラメータ数)
*/
template <class T> template <class Iter, class Cam> BlockDiagonalMatrix<T>
CameraCalibrator<T>::PlaneCost<Iter, Cam>::derivativeA(
    const Array<Cam>& cameras, const plane_type& plane, int j) const
{
    const auto&		data = corresListArray(j);
    derivative_type	J(ncameras());
    for (size_t i = 0; i < ncameras(); ++i)
    {
	J[i].resize(2 * data[i].size(), _adims[i]);
	size_t	k = 0;
	for (auto iter = data[i].begin(); iter != data[i].end(); ++iter)
	{
	    typename Cam::matrix_type	JJ;

	    cameras[i](plane(iter->first), &JJ);
	    J[i](k, 2, 0, _adims[i])
		= JJ(0, 2, 6 + _adims[0] - _adims[i], _adims[i]);
	    k += 2;
	}
    }
    
    return J;
}

//! 指定された参照平面について，平面パラメータに関する再投影誤差の1階微分を計算する．
/*!
  指定した参照平面に関わる#PlaneCost()で与えた全ての点対に対してヤコビ行列が計算される．
  \param cameras	カメラパラメータを収めた配列
  \param plane		参照平面
  \param j		参照平面を指定するindex
  \return		(2*点対数)x6ヤコビ行列
*/
template <class T>
template <class Iter, class Cam> typename CameraCalibrator<T>::matrix_type
CameraCalibrator<T>::PlaneCost<Iter, Cam>::derivativeB(
    const Array<Cam>& cameras, const plane_type& plane, int j) const
{
    const auto&	data = corresListArray(j);
    matrix_type	K(2 * _npoints[j], 6);
    for (size_t k = 0, i = 0; i < ncameras(); ++i)
	for (auto iter = data[i].begin(); iter != data[i].end(); ++iter)
	{
	    typename Cam::matrix_type	J;
	    cameras[i](plane(iter->first), &J);
	    slice<2, 6>(K, k, 0) = -slice<2, 3>(J, 0, 0)
				 * plane.derivative(iter->first);
	    k += 2;
	}
    
    return K;
}

//! 複数台のカメラのパラメータを更新する．
/*!
  \param cameras	更新されるカメラパラメータを収めた配列
  \param dcameras	更新量
*/
template <class T> template <class Iter, class Cam> void
CameraCalibrator<T>::PlaneCost<Iter, Cam>::updateA(
    Array<Cam>& cameras, const vector_type& dcameras) const
{
    size_t	d = 0;
    cameras[0].updateIntrinsic(dcameras(d, _adims[0]));
    d += _adims[0];
    for (size_t i = 1; i < _adims.size(); ++i)
    {
	if (_adims[i] == 6 + _adims[0])
	    cameras[i].update(dcameras(d, _adims[i]));
	else
	    cameras[i].updateFCC(dcameras(d, _adims[i]));
	d += _adims[i];
    }
}

//! 1枚の参照平面パラメータを更新する．
/*!
  \param plane		更新される参照平面パラメータ
  \param dplane		更新量
*/
template <class T> template <class Iter, class Cam> inline void
CameraCalibrator<T>::PlaneCost<Iter, Cam>::updateB(
    plane_type& plane, const vector_type& dplane) const
{
    plane.update(dplane);
}

//! 指定された参照平面について，全カメラに関する参照点とその投影像の対から成るデータを返す．
/*!
  \param j	参照平面を指定するindex
  \return	点対データ
*/
template <class T>
template <class Iter, class Cam> const typename Iter::value_type&
CameraCalibrator<T>::PlaneCost<Iter, Cam>::corresListArray(int j) const
{
    auto	iter = _begin;
    while (--j >= 0)
	++iter;
    return *iter;
}

//! 全カメラ，全参照平面および全点対に渡る再投影誤差の平均値を返す．
/*!
  \param cameras	カメラパラメータを収めた配列
  \param planes		参照平面パラメータを収めた配列
  \return		再投影誤差の平均値
*/
    
template <class T>
template <class Iter, class Cam> typename CameraCalibrator<T>::element_type
CameraCalibrator<T>::PlaneCost<Iter, Cam>::reprojectionError(
    const Array<Cam>& cameras, const Array<plane_type>& planes) const
{
    element_type	sumerr = 0.0;
    size_t		npoints = 0;
    for (size_t j = 0; j < planes.size(); ++j)
    {
	const auto	err = operator ()(cameras, planes[j], j);
	sumerr  += square(err);
	npoints += _npoints[j];
    }

    return sqrt(sumerr / npoints);
}
    
//! カメラパラメータの共分散行列からその標準偏差を求める．
/*!
  \param cameras	カメラパラメータを収めた配列
  \param covariance	カメラパラメータの共分散行列
  \return		標準偏差を収めた(カメラ数)x(6+内部パラメータ数)行列
*/
template <class T>
template <class Iter, class Cam> typename CameraCalibrator<T>::matrix_type
CameraCalibrator<T>::PlaneCost<Iter, Cam>::standardDeviations(
    const Array<Cam>& cameras, const matrix_type& covariance) const
{
    std::cerr << "covariance: " << covariance.nrow() << 'x' << covariance.ncol()
	      << std::endl;
    
    matrix_type	stddevs(_adims.size(), 6 + _adims[0]);
    for (size_t i = 0, m = 0; i < stddevs.nrow(); ++i)
    {
	size_t	npoints = 0;
	for (auto iter = _begin; iter != _end; ++iter)
	    npoints += (*iter)[i].size();

	auto	deviations = stddevs[i];
	for (size_t n = 6 + _adims[0] - _adims[i]; n < stddevs.ncol(); ++n)
	{
	    deviations[n] = sqrt(covariance[m][m] / npoints);
	    ++m;
	}
	
	if (stddevs.ncol() > 9)	// aspectとskewがパラメータに含まれているか？
	{ // aspectとskewは内部パラメータ行列の(1, 1), (1, 2)成分そのものとして
	  // 実装されているので，焦点距離で割らねばならない．
	    deviations[9]  /= cameras[i].k();
	    deviations[10] /= cameras[i].k();
	}
    }

    return stddevs;
}
	
}
#endif	//! __CAMERACALIBRATOR_H
