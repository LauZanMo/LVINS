#pragma once

#include "lvins_icp/point_cloud.h"

namespace lvins {

/**
 * @brief 体素网格采样
 * @details 该滤波器用于对点云进行下采样，实现逻辑：<br/>
 *          1. 根据输入点云，计算一个刚好包裹住该点云的立方体<br/>
 *          2. 根据设定的体素尺寸，将1中的大立方体分割成若干小立方体<br/>
 *          3. 对于每一个小立方体内的点云，计算其质心，并使用该质心代替该立方体内的点云
 * @param point_cloud 输入点云
 * @param voxel_grid_size 体素网格尺寸（m）
 * @return 处理后的点云
 */
RawPointCloud::Ptr voxelGridSample(const RawPointCloud::ConstPtr &point_cloud, float voxel_grid_size);

/**
 * @brief 自适应体素网格采样
 * @details 循环调用体素网格采样，并以两倍放大体素网格尺寸，直到点云数量小于期望值
 * @note 主要为低算力平台设计，以限制单次点云处理时间
 * @param point_cloud 输入点云
 * @param init_voxel_grid_size 初始体素网格尺寸（m）
 * @param max_voxel_grid_size 最大体素网格尺寸（m）
 * @param desire_point_cloud_size 期望点云数量
 * @param final_voxel_grid_size 最终体素网格尺寸（m）
 * @return 处理后的点云
 */
RawPointCloud::Ptr adaptiveVoxelGridSample(const RawPointCloud::ConstPtr &point_cloud, float init_voxel_grid_size,
                                           float max_voxel_grid_size, size_t desire_point_cloud_size,
                                           float *final_voxel_grid_size);

} // namespace lvins
