#include "lvins_icp/align/point_cloud_aligner.h"
#include "lvins_icp/factor/gicp_factor.h"

namespace lvins {

template class PointCloudAligner<GICPFactor>;

} // namespace lvins
