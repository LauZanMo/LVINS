#include "lvins_icp/preprocess/copy.h"

namespace lvins {

PointCloud::Ptr copy(const RawPointCloud &point_cloud) {
    return PointCloud::clone(point_cloud);
}

} // namespace lvins
