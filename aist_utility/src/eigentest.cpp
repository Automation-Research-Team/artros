namespace aist_utility
{
namespace Eigen
{
template <class T> void
doJob()
{
    using vector3_t	= ::Eigen::Matrix<T, 3, 1>;
    using matrix33_t	= ::Eigen::Matrix<T, 3, 3>;
    using angleaxis_t	= ::Eigen::AngleAxis<T>;

    std::cerr << "    translation >> ";
    vector3_t	t;
    std::cin >> t(0) >> t(1) >> t(2);

    std::cerr << "rotation vector >> ";
    vector3_t	theta;
    std::cin >> theta(0) >> theta(1) >> theta(2);

    std::cerr << "          scale >> ";
    T			scale;
    std::cin >> scale;

    const auto		q = angleaxis_t(theta(2), vector3_t::UnitZ())
			  * angleaxis_t(theta(1), vector3_t::UnitY())
			  * angleaxis_t(theta(0), vector3_t::UnitX());
    Similarity<T, 3>	transform(t, scale*q.toRotationMatrix());

    std::vector<std::pair<vector3_t, vector3_t> >	pairs;
    for (;;)
    {
	std::cerr << "point >> ";
	vector3_t	x;
	if (!(std::cin >> x(0) >> x(1) >> x(2)))
	    break;

	pairs.push_back(std::make_pair(x, transform(x)));
    }

    Similarity<T, 3>	transform_estimated(pairs.begin(), pairs.end());

    std::cerr << "--- transform(original) ---" << std::endl;
    std::cout << transform;
    std::cerr << "--- transform(estimated) ---" << std::endl;
    std::cout << transform_estimated;
    std::cerr << "scale = ";
    std::cout << transform_estimated.s() << std::endl;
}

}
}

int
main()
{
    TU::doJob<double>();
}
