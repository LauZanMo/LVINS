#include "lvins_icp/ann/flat_container.h"

namespace lvins {

std::string FlatContainer::Setting::print() const {
    return LVINS_FORMAT("  Container: Flat\n"
                        "    min squared distance in cell = {}\n"
                        "    min points number in cell = {}",
                        min_sq_dist_in_cell, max_num_points_in_cell);
}

size_t FlatContainer::size() const {
    return points_.size();
}

const std::vector<Eigen::Vector3f> &FlatContainer::points() const {
    return points_;
}

const std::vector<Eigen::Vector3f> &FlatContainer::normals() const {
    return normals_;
}

const std::vector<Eigen::Matrix3f> &FlatContainer::covariances() const {
    return covariances_;
}

void FlatContainer::add(const Setting &setting, const PointCloud &point_cloud, size_t i, const SE3f &T) {
    // 容器点集数超过阈值则直接返回
    if (points_.size() >= setting.max_num_points_in_cell) {
        return;
    }

    // 准备加入的点与容器点集中任意点的距离平方小于阈值则直接返回
    Eigen::Vector3f trans_point = T.cast<float>() * point_cloud[i].getVector3fMap();
    if (std::any_of(points_.begin(), points_.end(), [&](const auto &point) {
            return (point - trans_point).squaredNorm() < setting.min_sq_dist_in_cell;
        })) {
        return;
    }

    // 将点加入容器
    points_.push_back(std::move(trans_point));
    normals_.emplace_back(T.so3().cast<float>() * point_cloud[i].getNormalVector3fMap());
    covariances_.emplace_back(T.so3().matrix().cast<float>() * point_cloud[i].getCovariance3fMap() *
                              T.so3().matrix().transpose().cast<float>());
}

} // namespace lvins
