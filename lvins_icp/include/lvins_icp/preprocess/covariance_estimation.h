#pragma once

#include "lvins_icp/ann/nearest_neighbor_searcher.h"
#include "lvins_icp/point_cloud.h"

namespace lvins {

/**
 * @brief 估计点云协方差
 * @param point_cloud 输入点云
 * @param nn_searcher 最近邻搜索器
 * @param num_neighbors 用于协方差估计的近邻数量
 */
void estimateCovariance(PointCloud &point_cloud, NearestNeighborSearcher &nn_searcher, size_t num_neighbors);

} // namespace lvins
