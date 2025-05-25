#include "lvins_icp/align/reducer.h"
#include "lvins_icp/factor/gicp_factor.h"

namespace lvins::point_cloud_align {

// GICP因子的规约器方法显式实例化
template std::tuple<MatXf, VecXf, Float> Reducer::linearize<>(const NearestNeighborSearcher &, const PointCloud &,
                                                              const SE3f &, const SE3f &, std::vector<GICPFactor> &);
template Float Reducer::error<>(const NearestNeighborSearcher &, const PointCloud &, const SE3f &, const SE3f &,
                                std::vector<GICPFactor> &);

} // namespace lvins::point_cloud_align
