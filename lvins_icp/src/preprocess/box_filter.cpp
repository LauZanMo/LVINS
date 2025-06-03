#include "lvins_icp/preprocess/box_filter.h"

namespace lvins {

RawPointCloud::Ptr boxFilter(const RawPointCloud::ConstPtr &point_cloud, float crop_box_size) {
    const auto function = [crop_box_size](const Point &point) {
        // clang-format off
        return point.x() < -crop_box_size || point.x() > crop_box_size ||
               point.y() < -crop_box_size || point.y() > crop_box_size ||
               point.z() < -crop_box_size || point.z() > crop_box_size;
        // clang-format on
    };

    return gtsam_points::filter(point_cloud, function);
}

} // namespace lvins
