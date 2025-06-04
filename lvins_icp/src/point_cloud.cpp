#include "lvins_icp/point_cloud.h"

namespace lvins {

PointCloud::Ptr transform(const PointCloud &point_cloud, const SE3f &T) {
    const Eigen::Matrix4d transform = T.matrix().cast<double>();

    const auto ret = PointCloud::clone(point_cloud);
    for (size_t i = 0; i < ret->size(); ++i) {
        // 点云
        ret->points[i] = transform * ret->points[i];

        // 法向量
        if (point_cloud.normals) {
            ret->normals[i] = transform * ret->normals[i];
        }

        // 协方差
        if (point_cloud.covs) {
            ret->covs[i] = transform * ret->covs[i] * transform.transpose();
        }
    }

    return ret;
}

} // namespace lvins
