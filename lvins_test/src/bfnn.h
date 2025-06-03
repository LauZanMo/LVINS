//
// Created by xiang on 2021/8/18.
//

#ifndef SLAM_IN_AUTO_DRIVING_BFNN_H
#define SLAM_IN_AUTO_DRIVING_BFNN_H

#include "lvins_common/eigen_types.h"
#include "lvins_icp/point_cloud.h"

#include <thread>

namespace lvins {

/**
 * Brute-force Nearest Neighbour
 * @param cloud 点云
 * @param point 待查找点
 * @return 找到的最近点索引
 */
int bfnn_point(const PointCloud::Ptr &cloud, const Vec3f &point);

/**
 * Brute-force Nearest Neighbour, k近邻
 * @param cloud 点云
 * @param point 待查找点
 * @param k 近邻数
 * @return 找到的最近点索引
 */
std::vector<int> bfnn_point_k(const PointCloud::Ptr &cloud, const Vec3f &point, int k = 5);

/**
 * 对点云进行BF最近邻
 * @param cloud1  目标点云
 * @param cloud2  被查找点云
 * @param matches 两个点云内的匹配关系
 * @return
 */
void bfnn_cloud(const PointCloud::Ptr &cloud1, const PointCloud::Ptr &cloud2,
                std::vector<std::pair<size_t, size_t>> &matches);

/**
 * 对点云进行BF最近邻 多线程版本
 * @param cloud1
 * @param cloud2
 * @param matches
 */
void bfnn_cloud_mt(const PointCloud::Ptr &cloud1, const PointCloud::Ptr &cloud2,
                   std::vector<std::pair<size_t, size_t>> &matches);

/**
 * 对点云进行BF最近邻 多线程版本，k近邻
 * @param cloud1
 * @param cloud2
 * @param matches
 */
void bfnn_cloud_mt_k(const PointCloud::Ptr &cloud1, const PointCloud::Ptr &cloud2,
                     std::vector<std::pair<size_t, size_t>> &matches, int k = 5);
} // namespace lvins

#endif // SLAM_IN_AUTO_DRIVING_BFNN_H
