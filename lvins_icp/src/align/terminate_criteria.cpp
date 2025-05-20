#include "lvins_icp/align/terminate_criteria.h"

namespace lvins::point_cloud_align {

TerminateCriteria::TerminateCriteria(Float trans_eps, Float rot_eps) : trans_eps_(trans_eps), rot_eps_(rot_eps) {}

bool TerminateCriteria::isConverged(const VecXf &delta) const {
    bool converged = false;
    for (long i = 0; i < delta.rows(); i += 6) {
        // TODO: 记录收敛速度，防止长期无法收敛
        converged |= delta.segment<3>(i).norm() <= trans_eps_;
        converged |= delta.segment<3>(i + 3).norm() <= rot_eps_;
    }
    return converged;
}

} // namespace lvins::point_cloud_align
