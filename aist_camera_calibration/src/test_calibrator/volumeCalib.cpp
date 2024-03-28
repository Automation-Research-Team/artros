/*
 *  $Id$
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
using element_type	= double;
using point2_t		= Point2<element_type>;
using point3_t		= Point3<element_type>;
using corres22_t	= std::pair<point2_t, point2_t>;
using corres32_t	= std::pair<point3_t, point2_t>;
template <class CORRES>
using correses_t	= std::vector<CORRES>;
using camera_t		= TU::Camera<IntrinsicWithDistortion<
					 Intrinsic<element_type> > >;

/************************************************************************
*  static functions							*
************************************************************************/
//! 入力ストリームから参照点とその投影像の対を読み込む(ASCII)．
template <class P> inline static std::istream&
operator >>(std::istream& in, std::pair<P, point2_t>& corres)
{
    return in >> corres.first >> corres.second;
}

//! 出力ストリームに参照点とその投影像の対を書き出す(ASCII)．
template <class P> inline static std::ostream&
operator <<(std::ostream& out, const std::pair<P, point2_t>& corres)
{
    return corres.second.put(corres.first.put(out));
}

//! 入力ストリームからリストに読み込む(ASCII)．
template <class T> static std::istream&
operator >>(std::istream& in, std::vector<T>& v)
{
    for (char c; in.get(c); )
	if (!isspace(c))
	{
	    in.putback(c);
	    T	element;
	    in >> element;
	    v.push_back(element);
	}

    return in;
}

//! 出力ストリームにリストの要素を全て書き出す(ASCII)．
template <class T> inline static std::ostream&
operator <<(std::ostream& out, const std::vector<T>& v)
{
    std::copy(v.begin(), v.end(), std::ostream_iterator<T>(out));
    return out << std::endl;
}

//! コマンドの使い方を表示する．
static void
usage(const char* s)
{
    using namespace	std;

    cerr << "\nCalibrate one or more cameras with 3D reference points.\n"
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

template <class Cam> void
doCalibration(const correses_t<corres32_t>& correses, bool refine)
{
    CameraCalibrator<element_type>	calibrator;
    Cam					camera;

    calibrator.volumeCalib(correses.cbegin(), correses.cend(),
			   camera, refine);
    printEstimatedCamera(std::cerr, camera, calibrator.standardDeviations());

    std::cerr << "--- Reprojection error ---\n  "
	      << calibrator.reprojectionError() << '\n'
	      << std::endl;
    std::cout << camera.P() << camera.d1() << ' ' << camera.d2() << '\n'
	      << std::endl;
}

//! 出力ストリームに推定されたカメラのパラメータ値とその標準偏差を書き出す．
template <class Cam> std::ostream&
printEstimatedCamera(std::ostream& out, const Cam& camera,
		     const typename Cam::matrix_type& stddevs)
{
    using namespace	std;
    using element_type	= typename Cam::element_type;

    if (stddevs.nrow() == 0)
    {
	out << "--- camera ---\n" << camera << endl;
    }
    else
    {
	out << "--- stddevs.nrow(): " << stddevs.nrow() << " ---\n";

	out << camera.Pc();

	const auto	DEG = element_type(180) / element_type(M_PI);
	const auto&	devs = stddevs[0];
	const auto	t = camera.t();
	const auto	theta = rotation_axis(camera.Rt());
	out << "--- camera ---\n"
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
    bool		estimateDistortion = false;
    bool		refine = false;
    extern char*	optarg;
    extern int		optind;

  // Parse command line.
    for (int c; (c = getopt(argc, argv, "drh")) != EOF; )
	switch (c)
	{
	  case 'd':
	    estimateDistortion = true;
	    break;
	  case 'r':
	    refine = true;
	    break;

	  case 'h':
	    TU::usage(argv[0]);
	    return 1;
	}

  // Read the input PAIR file.
    correses_t<corres32_t>	correses;
    cin >> correses;
    std::cerr << correses.size() << " correspondences" << std::endl;
    cerr << correses;

  // Do main job.
    try
    {
	if (estimateDistortion)
	    doCalibration<camera_with_distortion_type>(correses, true);
	else
	    doCalibration<camera_type>(correses, refine);
    }
    catch (exception& err)
    {
	cerr << err.what() << endl;
	return 1;
    }

    return 0;
}
