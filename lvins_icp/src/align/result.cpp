#include "lvins_icp/align/result.h"

namespace lvins::point_cloud_align {

Result::Result(const SE3f &init_T_tb, const std::vector<SE3f> &init_T_bs) : T_tb(init_T_tb), T_bs(init_T_bs) {
    const long dim = 6 + static_cast<long>(T_bs.size()) * 6;
    H              = MatXd::Zero(dim, dim);
    b              = VecXd::Zero(dim);
}

} // namespace lvins::point_cloud_align
