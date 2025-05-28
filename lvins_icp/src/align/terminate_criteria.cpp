#include "lvins_icp/align/terminate_criteria.h"
#include "lvins_common/logger.h"

namespace lvins::point_cloud_align {

TerminateCriteria::TerminateCriteria(double trans_eps, double rot_eps) : trans_eps_(trans_eps), rot_eps_(rot_eps) {}

bool TerminateCriteria::isConverged(const VecXd &delta) const {
    LVINS_CHECK(delta.rows() % 6 == 0, "delta rows should be multiple of 6!");
    for (long i = 0; i < delta.rows(); i += 6) {
        // TODO: 记录收敛速度，防止长期无法收敛
        if (delta.segment<3>(i).norm() > trans_eps_ || delta.segment<3>(i + 3).norm() > rot_eps_) {
            return false;
        }
    }
    return true;
}

double TerminateCriteria::transEpsilon() const {
    return trans_eps_;
}

double TerminateCriteria::rotEpsilon() const {
    return rot_eps_;
}

std::string TerminateCriteria::print() const {
    return LVINS_FORMAT("  Terminate criteria:\n"
                        "    translation epsilon = {}\n"
                        "    rotation epsilon = {}",
                        trans_eps_, rot_eps_);
}

} // namespace lvins::point_cloud_align
