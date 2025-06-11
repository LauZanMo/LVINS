#pragma once

#include "lvins_icp/point_cloud.h"

namespace lvins {

/**
 * @brief 移除无效点
 * @details 根据有效性标志位移除点云中的无效点
 * @param point_cloud 输入的点云
 * @param valid 有效性标志位，1表示有效，0表示无效
 * @return 移除无效点后的点云
 */
PointCloud::Ptr removeInvalidPoints(const PointCloud::ConstPtr &point_cloud, const std::vector<uint8_t> &valid);

} // namespace lvins
