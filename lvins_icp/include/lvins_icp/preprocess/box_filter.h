#pragma once

#include "lvins_icp/point_cloud.h"

namespace lvins {

/**
 * @brief 载体滤波
 * @details 该滤波器用于滤除由于载体自身遮挡而获得的点云，实现逻辑：<br/>
 *          对点云进行遍历，若点云在以自身为中心，边长为2倍裁剪尺寸的立方体内，则进行滤除
 * @param point_cloud 输入点云
 * @param crop_box_size 裁剪尺寸（m）
 * @return 处理后的点云
 */
RawPointCloud::Ptr boxFilter(const RawPointCloud::ConstPtr &point_cloud, float crop_box_size);

} // namespace lvins
