#include "lvins_icp/preprocess/covariance_estimation.h"

#include <tbb/parallel_for.h>

namespace lvins {

void estimateCovariance(PointCloud &point_cloud, NearestNeighborSearcher &nn_searcher, size_t num_neighbors) {
    // 嵌入点云
    nn_searcher.insert(point_cloud, SE3f(Mat44f::Identity()));

    const Eigen::DiagonalMatrix<float, 3> diag(1e-3, 1.0, 1.0);
    tbb::parallel_for(tbb::blocked_range<size_t>(0, point_cloud.size()), [&](const tbb::blocked_range<size_t> &range) {
        for (size_t i = range.begin(); i < range.end(); ++i) {
            // 最近邻搜索
            std::vector<size_t> k_indices(num_neighbors);
            std::vector<float> k_sq_dists(num_neighbors);
            const auto n =
                    nn_searcher.knnSearch(point_cloud[i].getVector3fMap(), num_neighbors, k_indices, k_sq_dists, 1.0);

            // 如果邻域点数小于5，则设置协方差为单位矩阵
            if (n < 5) {
                point_cloud[i].getCovariance3fMap().setIdentity();
                continue;
            }

            // 计算中间值
            Eigen::Vector3f sum_points = Eigen::Vector3f::Zero();
            Eigen::Matrix3f sum_cross  = Eigen::Matrix3f::Zero();
            for (size_t j = 0; j < n; ++j) {
                const auto &pt = nn_searcher.point(k_indices[j]);
                sum_points += pt;
                sum_cross += pt * pt.transpose();
            }
            const Eigen::Vector3f mean = sum_points / n;
            const Eigen::Matrix3f cov  = (sum_cross - mean * sum_points.transpose()) / n;

            // 计算特征值和特征向量
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver;
            solver.computeDirect(cov);

            // 计算协方差
            point_cloud[i].getCovariance3fMap() = solver.eigenvectors() * diag * solver.eigenvectors().transpose();
        }
    });
}

} // namespace lvins
