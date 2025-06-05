#include "lvins_icp/align/result.h"

namespace lvins::point_cloud_align {

std::string Result::print() const {
    std::string ret;
    auto end = LVINS_FORMAT_TO(std::back_inserter(ret),
                               "Point cloud align result:"
                               "\n  T_tb = {}",
                               LVINS_VECTOR_FMT(T_tb.params()));
    for (size_t i = 0; i < T_bs.size(); ++i) {
        end = LVINS_FORMAT_TO(end, "\n  T_bs[{}] = {}", i, LVINS_VECTOR_FMT(T_bs[i].params()));
    }
    LVINS_FORMAT_TO(end, "\n  iterations = {}", iterations);
    return ret;
}

} // namespace lvins::point_cloud_align
