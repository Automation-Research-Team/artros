/*
 *  $Id$  
 */
/*!
  \mainpage	planeCalib - 既知の平面パターンを1台もしくはそれ以上のカメラに提示することによってカメラの内部・外部パラメータを推定するプロブラム
  \anchor	planeCalib

  \section functions 機能

  本プログラムの基本的な機能は，#markerDetector によって求められた参照
  点とその投影像であるマーカ点の2次元座標対のリストを入力し，それらから
  全カメラの内部・外部パラメータを推定することである．

  - [<b>機能</b>] 参照点とその投影像であるマーカ点の2次元座標対のリストを標準入
	力から入力し，全カメラの内部・外部パラメータを推定してそれらを
	標準出力に出力する．
  - [<b>入力</b>] 参照点とその投影像であるマーカ点の2次元座標対のリストであり，
	次のような形式を持つ:
	\f[
	  \begin{array}{c}
	    \begin{array}{ccccccccc}
	      x_{11}^1 & y_{11}^1 & u_{11}^1 & v_{11}^1 &
	      x_{12}^1 & y_{12}^1 & u_{12}^1 & v_{12}^1 & \cdots \\
	      x_{21}^1 & y_{21}^1 & u_{21}^1 & v_{21}^1 &
	      x_{22}^1 & y_{22}^1 & u_{22}^1 & v_{22}^1 & \cdots \\
	      & & & & \vdots \\
	      x_{I1}^1 & y_{I1}^1 & u_{I1}^1 & v_{I1}^1 &
	      x_{I2}^1 & y_{I2}^1 & u_{I2}^1 & v_{I2}^1 & \cdots
	    \end{array} \\
	    \mbox{(blank line)}\\
	    \begin{array}{ccccccccc}
	      x_{11}^2 & y_{11}^2 & u_{11}^2 & v_{11}^2 &
	      x_{12}^2 & y_{12}^2 & u_{12}^2 & v_{12}^2 & \cdots \\
	      x_{21}^2 & y_{21}^2 & u_{21}^2 & v_{21}^2 &
	      x_{22}^2 & y_{22}^2 & u_{22}^2 & v_{22}^2 & \cdots \\
	      & & & & \vdots \\
	      x_{I1}^2 & y_{I1}^2 & u_{I1}^2 & v_{I1}^2 &
	      x_{I2}^2 & y_{I2}^2 & u_{I2}^2 & v_{I2}^2 & \cdots
	    \end{array} \\
	    \mbox{(blank line)} \\
	    \vdots
	  \end{array}
	\f]
	全体は空行で区切られた複数のブロックから成り，各ブロックには1
	枚の参照平面を複数のカメラで観測して得られたデータが収められて
	いる．よって，ブロック数は提示した参照平面数に等しい．本コマン
	ドが実装しているアルゴリズムは，3枚以上の参照平面を必要とする．
	したがって，入力データに含まれるブロック数は3以上でなければなら
	ず，これが満たされなければエラーメッセージを表示して実行を中断
	する．\n
	1つのブロックは複数の行から成り，各行には1枚の参照平面を1つのカ
	メラで観測して得られたデータが収められている．よって，行数はそ
	の参照平面を観測したカメラ数に等しい．本コマンドが実装している
	アルゴリズムは，全参照平面が全カメラによって観測されていること
	を必要とする．したがって，1ブロックあたりの行数は全ブロックに渡っ
	て等しくなければならず，これが満たされなければエラーメッセージ
	を表示して実行を中断する．\n
	1つの行は4N個の数値から成る(Nは観測されたマーカ点数)．4つの数値
	が成す1つの組\f$x_{in}^j~y_{in}^j~u_{in}^j~v_{in}^j\f$はj番目の
	参照平面に固定された2次元座標系における参照点\f$(x_{in}^j,
	y_{in}^j)\f$がi番目のカメラによってマーカ点\f$(u_{in}^j,
	v_{in}^j)\f$として画像上で観測されたことを表し，添字nはそのよう
	な点対のうちn番目のものであることを示している．検出されるマーカ
	点の個数はカメラや参照平面によって異なるので，1行に含まれる数値
	の個数は一定ではない．

  - [<b>出力</b>] 本コマンドではカメラを放射歪曲を伴う透視投影としてモ
	デル化し，3次元空間中の点Xとその投影像uの関係を以下のような3ス
	テップの変換で表現する:
	\f[
	  \TUbeginarray{c} \TUvec{u}{} \\ 1 \TUendarray =
	  \TUvec{K}{}
	  \TUbeginarray{c} \TUbreve{x}{} \\ 1 \TUendarray
	   ,{\hskip 1em}
	  \TUbreve{x}{} = (1 + d_1\TUnorm{\TUvec{x}{}}^2 +
	   d_2\TUnorm{\TUvec{x}{}}^4)\TUvec{x}{}
	   ,{\hskip 1em}
	  \TUbeginarray{c} \TUvec{x}{} \\ 1 \TUendarray \simeq
	  \TUtvec{R}{}
	  \TUbeginarray{cc}
	    \TUvec{I}{3\times 3} & -\TUvec{t}{}
	  \TUendarray
	  \TUbeginarray{c} \TUvec{X}{} \\ 1 \TUendarray
	\f]
	ここで，tとRはそれぞれカメラの位置と姿勢を表す3次元ベクトルと
	3x3直交行列であり，d1, d2は放射歪曲係数である．さらにKは内部パ
	ラメータを表す3x3上半三角行列であり，
	\f[
	  \TUvec{K}{} =
	  \TUbeginarray{ccc}
	    ak & sk & u_0 \\ & k & v_0 \\ & & 1
	  \TUendarray
	\f]
	なる形を持つ．ここで，k, (u0, v0), a, s は，それぞれ焦点距離,
	画像主点，アスペクト比および非直交歪みである．\n
	本コマンドは，ワールド座標系を1番目のカメラ(入力データの各ブロッ
	クの1行目に相当するカメラ)のカメラ座標系に一致するようにとる．
	すなわち，第1カメラについては，\f$\TUvec{t}{} =
	\TUvec{0}{}\f$, \f$\TUvec{R}{} = \TUvec{I}{3\times 3}\f$とする．\n
	本コマンドは，次のような形式で各カメラのパラメータを出力する:
	\f[
	  \begin{array}{l}
	    \begin{array}{cccc}
	    P_{11} & P_{12} & P_{13} & P_{14} \\
	    P_{21} & P_{22} & P_{23} & P_{24} \\
	    P_{31} & P_{32} & P_{33} & P_{34}
	    \end{array} \\
	    \mbox{(blank line)} \\
	    \begin{array}{cc}
	      d_1 & d_2
	    \end{array} \\
	    \mbox{(blank line)}
	  \end{array}
	\f]
	ここで，Pijはカメラの3x4投影行列
	\f[
	  \TUvec{P}{} =
	  \TUvec{K}{}\TUtvec{R}{}
	  \TUbeginarray{cc}
	    \TUvec{I}{3\times 3} & -\TUvec{t}{}
	  \TUendarray
	 \f]
	 のij成分である．
  - [<b>コマンド呼び出しの形式</b>] 与えられた座標対のリスト<tt>markers.dat</tt>
	に対して
	\verbatim
  planeCalib [-r] [-d] < markers.dat > cameras.calib
	\endverbatim
	とすることにより，推定されたカメラパラメータが<tt>cameras.calib</tt>に
	書き込まれる．
  - [<b>コマンドオプション</b>] 本コマンドは，以下のオプションを持つ:
    - [<tt>-r</tt>] 線形計算によって求められたカメラパラメータを，非線形
	    最適化によってさらに高精度化する．ただし，カメラは透視投影
	    に従うものとし，レンズの放射歪曲の影響は考慮しない．
    - [<tt>-d</tt>] 線形計算によって求められたカメラパラメータを，非線形
	    最適化によってさらに高精度化する．透視投影の効果に加え，レ
	    ンズの放射歪曲の影響も考慮する．
  - [<b>補助出力</b>] 標準出力に推定されたカメラパラメータを出力すると
	同時に，それらをより分かりやすい形で標準エラー出力に出力する．
	<tt>-r</tt>または<tt>-d</tt>オプションを指定して非線形最適化を
	行う場合には，各パラメータ推定値の標準偏差もパラメータと共に括
	弧で囲んで出力する．さらに，推定されたパラメータを用いて参照点
	を各カメラの画像面に投影したときの平均再投影誤差も表示する．

  \file		main.cpp
  \brief	メイン関数
*/
#include <unistd.h>
#include <fstream>
#include <exception>
#include <list>
#include <iomanip>
#include "CameraCalibrator.h"
#include "TU/Camera++.h"

//! 本プログラムで定義されたクラスおよび関数を収める名前空間
namespace TU
{
/************************************************************************
*  typedefs								*
************************************************************************/
typedef double				element_type;
//! 参照平面上の参照点とその投影像の2次元座標対
typedef std::pair<Point2<element_type>, Point2<element_type> >	Corres;
//! 1枚の参照平面と1台のカメラ間で対応づけられた全ての#Corresから成るリスト
typedef std::list<Corres>		CorresList;
//! 1枚の参照平面に対して，全カメラの#CorresListを収めた配列
typedef Array<CorresList>		CorresListArray;
//! 全参照平面の#CorresListArrayを収めたリスト
typedef std::list<CorresListArray>	CorresListArrayList;
    
/************************************************************************
*  static functions							*
************************************************************************/
//! 入力ストリームから参照点とその投影像の対を読み込む(ASCII)．
inline static std::istream&
operator >>(std::istream& in, Corres& corres)
{
    return in >> corres.first >> corres.second;
}
    
//! 出力ストリームに参照点とその投影像の対を書き出す(ASCII)．
inline static std::ostream&
operator <<(std::ostream& out, const Corres& corres)
{
    return corres.second.put(corres.first.put(out));
}

//! 入力ストリームからリストに読み込む(ASCII)．
template <class T> static std::istream&
operator >>(std::istream& in, std::list<T>& list)
{
    for (char c; in.get(c); )
	if (c == '\n')
	    break;
	else if (!isspace(c))
	{
	    in.putback(c);
	    T	element;
	    in >> element;
	    list.push_back(element);
	}

    return in;
}

//! 出力ストリームにリストの要素を全て書き出す(ASCII)．
template <class T> inline static std::ostream&
operator <<(std::ostream& out, const std::list<T>& list)
{
    using namespace	std;
    
    copy(list.begin(), list.end(), ostream_iterator<T>(out));
    return out << endl;
}

//! コマンドの使い方を表示する．
static void
usage(const char* s)
{
    using namespace	std;
    
    cerr << "\nCalibrate one or more cameras with multiple planes.\n"
	 << endl;
    cerr << " Usage: " << s << " [-o <planeFile>] [options] < <markerFile>\n"
	 << endl;
    cerr << " Algorithm options.\n"
	 << "  -r:             do refinement by bundle adjustment.\n"
	 << "  -d:             estimate distortion.\n"
	 << endl;
    cerr << " Other options.\n"
	 << "  -o <planeFile>: output file for storing plane parameters.\n"
	 << "  -h:             print this\n"
	 << endl;
}

//! 出力ストリームに推定されたカメラのパラメータ値とその標準偏差を書き出す．
template <class Cam> std::ostream&
printEstimatedCameras(std::ostream& out, const Array<Cam>& cameras,
		      const typename Cam::matrix_type& stddevs)
{
    using namespace	std;
    using element_type	= typename Cam::element_type;

    if (stddevs.nrow() == 0)
    {
	for (size_t i = 0; i < cameras.size(); ++i)
	    out << "--- camera " << i << " ---\n"
		<< cameras[i]
		<< endl;
    }
    else
    {
	for (size_t i = 0; i < stddevs.nrow(); ++i)
	{
	    const auto	DEG = element_type(180) / element_type(M_PI);
	    const auto&	camera = cameras[i];
	    const auto	devs = stddevs[i];
	    const auto	t = camera.t();
	    const auto	theta = rotation_axis(camera.Rt());

	    out << camera.Pc();
	    
	    out << "--- camera " << i << " ---\n"
		<< "Position:        "
		<< camera.t()[0] << "(" << devs[0] << ") "
		<< camera.t()[1] << "(" << devs[1] << ") "
		<< camera.t()[2] << "(" << devs[2] << ")\n"
		<< "Rotation(deg.):  "
		<< DEG * theta[0] << "(" << DEG * devs[3] << ") "
		<< DEG * theta[1] << "(" << DEG * devs[4] << ") "
		<< DEG * theta[2] << "(" << DEG * devs[5] << ")\n"
		<< "Focal length:    "
		<< camera.k() << "(" << devs[6] << ")\n"
		<< "Principal point: "
		<< camera.u0()[0] << "(" << devs[7] << ") "
		<< camera.u0()[1] << "(" << devs[8] << ")\n"
		<< "Aspect ratio:    "
		<< camera.aspect() << "(" << devs[9] << ")\n"
		<< "Skew:            "
		<< camera.skew() << "(" << devs[10] << ")"
		<< endl;
	    if (stddevs.ncol() > 11)
		out <<   "d1:              "
		    << camera.d1() << "(" << devs[11] << ")\n"
		    <<   "d2:              "
		    << camera.d2() << "(" << devs[12] << ")"
		    << endl;
	    out << endl;
	}
    }

    return out;
}

//! 出力ストリームに推定された参照平面のパラメータ値を書き出す．
template <class T> std::ostream&
printEstimatedPlanes(std::ostream& out, const Array<ReferencePlane<T> >& planes)
{
    for (size_t j = 0; j < planes.size(); ++j)
	out << "--- plane " << j << " ---\n" << planes[j] << std::endl;
    
    return out;
}

}

/************************************************************************
*  global functions							*
************************************************************************/
//! メイン関数
int
main(int argc, char* argv[])
{
    using namespace	std;
    using namespace	TU;

    using camera_type		      = Camera<Intrinsic<element_type> >;
    using camera_with_distortion_type = Camera<IntrinsicWithDistortion<
						   Intrinsic<element_type> > >;
    
    void		usage(const char*);
    bool		refine = false, estimateDistortion = false,
			commonCenter = false;
    const char*		planeFile = 0;
    extern char*	optarg;
    extern int		optind;

  // Parse command line.
    for (int c; (c = getopt(argc, argv, "rdfo:h")) != EOF; )
	switch (c)
	{
	  case 'r':
	    refine = true;
	    break;
	  case 'd':
	    estimateDistortion = true;
	    break;
	  case 'f':
	    commonCenter = true;
	    break;
	  case 'o':
	    planeFile = optarg;
	    break;
	    
	  case 'h':
	    TU::usage(argv[0]);
	    return 1;
	}
    
  // Read the input PAIR file.
    CorresListArrayList	data;
    cin >> data;
    
  // Do main job.
    try
    {
	CameraCalibrator<element_type>				calib;
	Array<CameraCalibrator<element_type>::plane_type>	planes;

	if (estimateDistortion)
	{
	    Array<camera_with_distortion_type>	cameras;

	    planes = calib.planeCalib(data.begin(), data.end(), cameras,
				      commonCenter, true);

	    printEstimatedCameras(cerr, cameras, calib.standardDeviations());
	    printEstimatedPlanes(cerr, planes);
	    
	    cerr << "--- Reprojection error ---\n  "
		 << calib.reprojectionError() << '\n'
		 << endl;
	    for (size_t i = 0; i < cameras.size(); ++i)
		cout << cameras[i].P()
		     << cameras[i].d1() << ' ' << cameras[i].d2() << '\n'
		     << endl;
	}
	else
	{
	    Array<camera_type>	cameras;
	    
	    planes = calib.planeCalib(data.begin(), data.end(), cameras,
				      commonCenter, refine);

	    printEstimatedCameras(cerr, cameras, calib.standardDeviations());
	    printEstimatedPlanes(cerr, planes);

	    cerr << "--- Reprojection error ---\n  "
		 << calib.reprojectionError() << '\n'
		 << endl;
	    for (size_t i = 0; i < cameras.size(); ++i)
		cout << cameras[i].P()
		     << cameras[i].d1() << ' ' << cameras[i].d2() << '\n'
		     << endl;
	}

	if (planeFile)
	{
	    ofstream	out(planeFile);
	    if (!out)
		throw runtime_error("Cannot open output file for planes!!");
	    for (size_t j = 0; j < planes.size(); ++j)
		out << planes[j].Qt();
	}
    }
    catch (exception& err)
    {
	cerr << err.what() << endl;
	return 1;
    }
    
    return 0;
}

