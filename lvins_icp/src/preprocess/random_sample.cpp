#include "lvins_icp/preprocess/random_sample.h"

namespace lvins {

PointCloud::Ptr randomSample(const PointCloud::ConstPtr &point_cloud,
                             float sample_rate) {
  std::mt19937 mt;
  return gtsam_points::random_sampling(point_cloud, sample_rate, mt);
}

} // namespace lvins
