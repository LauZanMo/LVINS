#pragma once

#include "lvins_icp/point_cloud.h"

namespace lvins {

/**
 * @brief 拷贝点云
 * @param point_cloud 输入点云
 * @return 输出点云
 */
PointCloud::Ptr copy(const RawPointCloud &point_cloud);

} // namespace lvins
