#pragma once

#include "lvins_icp/point_cloud.h"

namespace lvins {

/**
 * @brief 拷贝点云（丢弃时间信息）
 * @param src 原始点云
 * @param output 输出点云
 */
void copy(const RawPointCloud &src, PointCloud &output);

} // namespace lvins
