#include "lvins_icp/preprocess/remove_invalid_points.h"
#include "lvins_common/logger.h"

namespace lvins {

PointCloud::Ptr removeInvalidPoints(const PointCloud::ConstPtr &point_cloud, const std::vector<uint8_t> &valid) {
    LVINS_CHECK(point_cloud->size() == valid.size(), "Point cloud size should match valid size!");
    return gtsam_points::filter_by_index(point_cloud, [&](size_t idx) {
        return valid[idx];
    });
}

} // namespace lvins
