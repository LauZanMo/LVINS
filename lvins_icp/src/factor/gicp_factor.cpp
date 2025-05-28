#include "lvins_icp/factor/gicp_factor.h"
#include "lvins_common/logger.h"

namespace lvins {

std::string GICPFactor::Setting::print() const {
    return LVINS_FORMAT("  Factor: GICP\n"
                        "    max search squared distance = {}",
                        max_search_sq_dist);
}

GICPFactor::GICPFactor(Setting *setting) : setting_(setting) {
    LVINS_CHECK(setting_, "Setting should not be nullptr!");
}

bool GICPFactor::linearize(const NearestNeighborSearcher &target_nn_searcher, const PointCloud &source,
                           const SE3f &T_tb, const SE3f &T_bs, size_t source_index, MatXd &H, VecXd &b, double &e) {
    source_index_ = source_index;
    target_index_ = std::numeric_limits<size_t>::max();

    // 最近邻搜索
    const SE3d T_ts             = (T_tb * T_bs).cast<double>();
    const Vec3d source_pt       = source[source_index_].getVector3fMap().cast<double>();
    const Vec3d trans_source_pt = T_ts * source_pt;
    std::vector<size_t> k_index;
    std::vector<float> k_sq_dist;
    if (!target_nn_searcher.knnSearch(trans_source_pt.cast<float>(), 1, k_index, k_sq_dist,
                                      setting_->max_search_sq_dist)) {
        return false;
    }
    target_index_ = k_index[0];

    // 计算残差
    const Vec3d residual = trans_source_pt - target_nn_searcher.point(target_index_).cast<double>();

    // 计算中间值
    const Mat33d T_tb_mat = T_tb.so3().matrix().cast<double>();
    const Mat33d T_ts_mat = T_ts.so3().matrix();

    // 计算雅可比矩阵
    MatXd J             = MatXd::Zero(3, 12);
    J.block<3, 3>(0, 0) = Mat33d::Identity();
    J.block<3, 3>(0, 3) = -T_tb_mat * SO3d::hat(T_bs.cast<double>() * source_pt);
    J.block<3, 3>(0, 6) = T_tb_mat;
    J.block<3, 3>(0, 9) = -T_ts_mat * SO3d::hat(source_pt);

    // 计算协方差矩阵
    point_cov_ = (target_nn_searcher.covariance(target_index_).cast<double>() +
                  T_ts_mat * source[source_index_].getCovariance3fMap().cast<double>() * T_ts_mat.transpose())
                         .inverse();

    // 计算线性化结果
    H = J.transpose() * point_cov_ * J;
    b = J.transpose() * point_cov_ * residual;
    e = 0.5 * residual.transpose() * point_cov_ * residual;

    return true;
}

double GICPFactor::error(const NearestNeighborSearcher &target_nn_searcher, const PointCloud &source, const SE3f &T_tb,
                         const SE3f &T_bs) const {
    // 未找到匹配点，返回0
    if (target_index_ == std::numeric_limits<size_t>::max()) {
        return 0.0;
    }

    // 计算误差
    const Vec3d trans_source_pt = (T_tb * T_bs).cast<double>() * source[source_index_].getVector3fMap().cast<double>();
    const Vec3d residual        = trans_source_pt - target_nn_searcher.point(target_index_).cast<double>();
    return 0.5 * residual.transpose() * point_cov_ * residual;
}

bool GICPFactor::isInlier() const {
    return target_index_ != std::numeric_limits<size_t>::max();
}

} // namespace lvins
