#include "lvins_common/logger.h"

namespace lvins::point_cloud_align {

template<typename Factor>
Result Optimizer::optimize(const NearestNeighborSearcher &target_nn_searcher,
                           const std::vector<const PointCloud *> &source_point_clouds, const SE3f &init_T_tb,
                           const std::vector<SE3f> &init_T_bs, bool estimate_extrinsic,
                           const TerminateCriteria &criteria, std::vector<std::vector<Factor>> &factors) const {
    double lambda = init_lambda_;
    Result result(init_T_tb, init_T_bs);

    // 优化循环
    for (size_t i = 0; i < max_iterations_ && !result.converged; ++i) {
        const long dim = 6 + static_cast<long>(init_T_bs.size()) * 6;
        MatXd H        = MatXd::Zero(dim, dim);
        VecXd b        = VecXd::Zero(dim);
        double e       = 0.0;

        // 线性化
        for (size_t j = 0; j < source_point_clouds.size(); ++j) {
            auto [Hj, bj, ej] = Reducer::linearize(target_nn_searcher, *source_point_clouds[j], result.T_tb,
                                                   result.T_bs[j], factors[j]);

            // 组装信息矩阵、信息向量并计算误差值
            const long o_ext = 6 + static_cast<long>(j) * 6;
            H.block<6, 6>(0, 0) += Hj.template block<6, 6>(0, 0);
            b.head<6>() += bj.template head<6>();
            if (estimate_extrinsic) {
                H.block<6, 6>(0, o_ext) += Hj.template block<6, 6>(0, 6);
                H.block<6, 6>(o_ext, 0) += Hj.template block<6, 6>(6, 0);
                H.block<6, 6>(o_ext, o_ext) += Hj.template block<6, 6>(6, 6);
                b.segment<6>(o_ext) += bj.template segment<6>(6);
            }
            e += ej;
        }

        // lambda迭代
        bool success = false;
        for (size_t j = 0; j < max_inner_iterations_; ++j) {
            // 带阻尼求解
            const VecXd delta = (H + lambda * MatXd::Identity(dim, dim)).ldlt().solve(-b);

            // 验证求解结果
            const SE3f new_T_tb =
                    result.T_tb * SE3f(SO3f::exp(delta.segment<3>(3).cast<Float>()), delta.head<3>().cast<Float>());
            std::vector<SE3f> new_T_bs;
            double new_e = 0.0;
            for (size_t k = 0; k < source_point_clouds.size(); ++k) {
                const long o_ext = 6 + static_cast<long>(k) * 6;
                new_T_bs.push_back(result.T_bs[k] * SE3f(SO3f::exp(delta.segment<3>(o_ext + 3).cast<Float>()),
                                                         delta.segment<3>(o_ext).cast<Float>()));
                new_e += Reducer::error(target_nn_searcher, *source_point_clouds[k], new_T_tb, new_T_bs[k], factors[k]);
            }

            LVINS_DEBUG("Point cloud align optimizer:\n"
                        "   iteration = {}\n"
                        "   inner_iteration = {}\n"
                        "   error = {}\n"
                        "   new error = {}\n"
                        "   lambda = {}\n"
                        "   dt = {}\n"
                        "   dr = {}",
                        i, j, e, new_e, lambda, delta.head<3>().norm(), delta.segment<3>(3).norm());

            // 判断是否收敛，并调整lambda
            if (new_e <= e) {
                result.converged = criteria.isConverged(delta);
                result.T_tb      = new_T_tb;
                result.T_bs      = new_T_bs;
                lambda /= lambda_factor_;
                success = true;
                break;
            }

            lambda *= lambda_factor_;
        }

        // 更新结果
        result.iterations = i;
        result.H          = H;
        result.b          = b;
        result.e          = e;

        // 收敛则退出
        if (!success) {
            break;
        }
    }

    // 计算采用点数量
    for (size_t i = 0; i < factors.size(); ++i) {
        result.num_inliers += std::count_if(factors[i].begin(), factors[i].end(), [](const auto &factor) {
            return factor.isInlier();
        });
    }

    return result;
}

template<typename Factor>
void Optimizer::linearize(const NearestNeighborSearcher &target_nn_searcher,
                          const std::vector<const PointCloud *> &source_point_clouds, const SE3f &T_tb,
                          const std::vector<SE3f> &T_bs, bool estimate_extrinsic, MatXd &H, VecXd &b,
                          std::vector<std::vector<Factor>> &factors) const {
    const long dim = 6 + static_cast<long>(T_bs.size()) * 6;
    H              = MatXd::Zero(dim, dim);
    b              = VecXd::Zero(dim);

    // 线性化
    for (size_t i = 0; i < source_point_clouds.size(); ++i) {
        auto [Hi, bi, ei] = Reducer::linearize(target_nn_searcher, *source_point_clouds[i], T_tb, T_bs[i], factors[i]);

        // 组装信息矩阵、信息向量
        const long o_ext = 6 + static_cast<long>(i) * 6;
        H.block<6, 6>(0, 0) += Hi.template block<6, 6>(0, 0);
        b.head<6>() += bi.template head<6>();
        if (estimate_extrinsic) {
            H.block<6, 6>(0, o_ext) += Hi.template block<6, 6>(0, 6);
            H.block<6, 6>(o_ext, 0) += Hi.template block<6, 6>(6, 0);
            H.block<6, 6>(o_ext, o_ext) += Hi.template block<6, 6>(6, 6);
            b.segment<6>(o_ext) += bi.template segment<6>(6);
        }
    }
}

} // namespace lvins::point_cloud_align
