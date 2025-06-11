#pragma once

#include "lvins_common/nav_state.h"
#include "lvins_icp/point_cloud.h"
#include "lvins_lidar/lidar_geometry_base.h"

namespace lvins {

/**
 * @brief 对传感器坐标下的点云进行运动矫正
 * @details 运动矫正总共分为四个步骤（步骤1-3为并行）<br/>
 *          1. 查找雷达点时间戳的两个相邻的导航状态，若无法查找到则跳过<br/>
 *          2. 对雷达点的探测范围进行检查，若不在探测范围内则跳过跳过<br/>
 *          3. 内插导航状态至雷达点时间戳处，对雷达点进行运动补偿<br/>
 *          4. 将点集转换为点云<br/>
 * @param point_cloud 原始点云
 * @param lidar 采集原始点云的雷达
 * @param T_bs body坐标系（通常是IMU）到传感器坐标系的变换
 * @param states 上一点云到当前点云间的导航状态集合
 * @return 运动矫正后的点云
 * @warning 并行处理后可能会存在点云顺序和之前不一致的问题
 */
PointCloud::Ptr deskew(const RawPointCloud &point_cloud, const LidarGeometryBase &lidar, const SE3f &T_bs,
                       const NavStates &states);

} // namespace lvins
