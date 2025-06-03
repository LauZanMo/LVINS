#include "lvins_icp/preprocess/covariance_estimation.h"
#include "lvins_common/logger.h"

#include <gtsam_points/features/covariance_estimation.hpp>

namespace lvins {

void estimateCovariance(PointCloud &point_cloud, size_t num_neighbors) {
    LVINS_CHECK(num_neighbors > 4, "Number of neighbors for covariance estimation should be greater than 4!");

    point_cloud.add_covs(gtsam_points::estimate_covariances(point_cloud, static_cast<int>(num_neighbors), 4));
}

} // namespace lvins
