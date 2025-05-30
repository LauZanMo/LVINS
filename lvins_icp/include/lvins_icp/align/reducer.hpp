#include <tbb/tbb.h>

namespace lvins::point_cloud_align {

/**
 * @brief 线性化点云配准因子求和类
 * @tparam Factor 点云配准因子类型
 */
template<typename Factor>
struct LinearizeSum {
    LinearizeSum(const NearestNeighborSearcher &target_nn_searcher, const PointCloud &source, const SE3f &T_tb,
                 const SE3f &T_bs, std::vector<Factor> &factors)
        : target_nn_searcher(target_nn_searcher), source(source), T_tb(T_tb), T_bs(T_bs), factors(factors) {}

    LinearizeSum(const LinearizeSum &other, tbb::split)
        : target_nn_searcher(other.target_nn_searcher),
          source(other.source),
          T_tb(other.T_tb),
          T_bs(other.T_bs),
          factors(other.factors) {}

    void operator()(const tbb::blocked_range<size_t> &r) {
        MatXd Ht  = H;
        VecXd bt  = b;
        double et = e;

        for (size_t i = r.begin(); i != r.end(); ++i) {
            MatXd Hi;
            VecXd bi;
            double ei;

            if (!factors[i].linearize(target_nn_searcher, source, T_tb, T_bs, i, Hi, bi, ei)) {
                continue;
            }

            Ht += Hi;
            bt += bi;
            et += ei;
        }

        H = Ht;
        b = bt;
        e = et;
    }

    void join(const LinearizeSum &other) {
        H += other.H;
        b += other.b;
        e += other.e;
    }

    const NearestNeighborSearcher &target_nn_searcher;
    const PointCloud &source;
    const SE3f &T_tb;
    const SE3f &T_bs;
    std::vector<Factor> &factors;

    MatXd H{MatXd::Zero(12, 12)};
    VecXd b{VecXd::Zero(12)};
    double e{0.0};
};

/**
 * @brief 误差求和类
 * @tparam Factor 点云配准因子类型
 */
template<typename Factor>
struct ErrorSum {
    ErrorSum(const NearestNeighborSearcher &target_nn_searcher, const PointCloud &source, const SE3f &T_tb,
             const SE3f &T_bs, std::vector<Factor> &factors)
        : target_nn_searcher(target_nn_searcher), source(source), T_tb(T_tb), T_bs(T_bs), factors(factors) {}

    ErrorSum(const ErrorSum &other, tbb::split)
        : target_nn_searcher(other.target_nn_searcher),
          source(other.source),
          T_tb(other.T_tb),
          T_bs(other.T_bs),
          factors(other.factors) {}

    void operator()(const tbb::blocked_range<size_t> &r) {
        double et = e;

        for (size_t i = r.begin(); i != r.end(); ++i) {
            et += factors[i].error(target_nn_searcher, source, T_tb, T_bs);
        }

        e = et;
    }

    void join(const ErrorSum &other) {
        e += other.e;
    }

    const NearestNeighborSearcher &target_nn_searcher;
    const PointCloud &source;
    const SE3f &T_tb;
    const SE3f &T_bs;
    std::vector<Factor> &factors;

    double e{0.0};
};

template<typename Factor>
std::tuple<MatXd, VecXd, double> Reducer::linearize(const NearestNeighborSearcher &target_nn_searcher,
                                                    const PointCloud &source, const SE3f &T_tb, const SE3f &T_bs,
                                                    std::vector<Factor> &factors) {
    LinearizeSum sum(target_nn_searcher, source, T_tb, T_bs, factors);
    tbb::parallel_reduce(tbb::blocked_range<size_t>(0, factors.size(), 8), sum);
    return {sum.H, sum.b, sum.e};
}

template<typename Factor>
double Reducer::error(const NearestNeighborSearcher &target_nn_searcher, const PointCloud &source, const SE3f &T_tb,
                      const SE3f &T_bs, std::vector<Factor> &factors) {
    ErrorSum<Factor> sum(target_nn_searcher, source, T_tb, T_bs, factors);
    tbb::parallel_reduce(tbb::blocked_range<size_t>(0, factors.size(), 8), sum);
    return sum.e;
}

} // namespace lvins::point_cloud_align
