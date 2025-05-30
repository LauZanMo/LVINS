#include "lvins_common/logger.h"

namespace lvins {

template<typename Factor>
PointCloudAligner<Factor>::PointCloudAligner(const Optimizer &optimizer, const TerminateCriteria &criteria,
                                             const FactorSetting &factor_setting)
    : PointCloudAlignerBase(optimizer, criteria), factor_setting_(factor_setting) {}

template<typename Factor>
PointCloudAlignerBase::Result
PointCloudAligner<Factor>::align(const NearestNeighborSearcher &target_nn_searcher,
                                 const std::vector<const PointCloud *> &source_point_clouds, const SE3f &init_T_tb,
                                 const std::vector<SE3f> &init_T_bs, bool estimate_extrinsic) const {
    LVINS_CHECK(source_point_clouds.size() == init_T_bs.size(),
                "The number of source point clouds ({}) should match the number of initial extrinsics ({})!",
                source_point_clouds.size(), init_T_bs.size());

    // 初始化因子
    std::vector<std::vector<Factor>> factors(source_point_clouds.size());
    for (size_t i = 0; i < source_point_clouds.size(); ++i) {
        factors[i].resize(source_point_clouds[i]->size(), Factor(&factor_setting_));
    }

    // 单点云配准约束不足，不支持估计外参
    estimate_extrinsic = source_point_clouds.size() > 1 ? estimate_extrinsic : false;

    // 优化并返回结果
    return optimizer_.optimize(target_nn_searcher, source_point_clouds, init_T_tb, init_T_bs, estimate_extrinsic,
                               criteria_, factors);
}

template<typename Factor>
void PointCloudAligner<Factor>::linearize(const NearestNeighborSearcher &target_nn_searcher,
                                          const std::vector<const PointCloud *> &source_point_clouds, const SE3f &T_tb,
                                          const std::vector<SE3f> &T_bs, bool estimate_extrinsic,
                                          std::vector<size_t> &num_inliers, MatXd &H, VecXd &b) const {
    LVINS_CHECK(source_point_clouds.size() == T_bs.size(),
                "The number of source point clouds ({}) should match the number of extrinsics ({})!",
                source_point_clouds.size(), T_bs.size());

    // 初始化因子
    std::vector<std::vector<Factor>> factors(source_point_clouds.size());
    for (size_t i = 0; i < source_point_clouds.size(); ++i) {
        factors[i].resize(source_point_clouds[i]->size(), Factor(&factor_setting_));
    }

    // 线性化
    optimizer_.linearize(target_nn_searcher, source_point_clouds, T_tb, T_bs, estimate_extrinsic, H, b, factors);

    // 计算每个点云的有效点数量
    num_inliers.resize(factors.size());
    for (size_t i = 0; i < factors.size(); ++i) {
        num_inliers[i] = std::count_if(factors[i].begin(), factors[i].end(), [](const auto &factor) {
            return factor.isInlier();
        });
    }
}

template<typename Factor>
const typename PointCloudAligner<Factor>::FactorSetting &PointCloudAligner<Factor>::factorSetting() const {
    return factor_setting_;
}

template<typename Factor>
std::string PointCloudAligner<Factor>::print() const {
    return LVINS_FORMAT("Point cloud aligner:\n"
                        "{}\n"
                        "{}\n"
                        "{}",
                        optimizer_.print(), criteria_.print(), factor_setting_.print());
}

} // namespace lvins