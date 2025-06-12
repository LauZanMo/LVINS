#pragma once

#include "lvins_icp/point_cloud.h"

namespace lvins {

/**
 * @brief 随机采样
 * @param point_cloud 输入点云
 * @param sample_rate 采样率
 * @return 采样点云
 */
PointCloud::Ptr randomSample(const PointCloud::ConstPtr &point_cloud, float sample_rate);

} // namespace lvins
