#pragma once

#include "lvins_icp/point_cloud.h"

#include <sensor_msgs/msg/point_cloud2.hpp>

namespace lvins {

///< Ouster原始点及点云类
struct EIGEN_ALIGN16 OusterPoint {
    PCL_ADD_POINT4D; ///< 三维点（齐次形式）
    float intensity; ///< 强度
    uint32_t t;      ///< Ouster：相对于扫描开始时间的差值（ns）
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
using OusterPointCloud = pcl::PointCloud<OusterPoint>;

///< Velodyne原始点及点云类
struct EIGEN_ALIGN16 VelodynePoint {
    PCL_ADD_POINT4D; ///< 三维点（齐次形式）
    float intensity; ///< 强度
    float time;      ///< Velodyne：相对于扫描开始时间的差值（s）
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
using VelodynePointCloud = pcl::PointCloud<VelodynePoint>;

///< Livox原始点及点云类
using LivoxPoint      = RawPoint;
using LivoxPointCloud = RawPointCloud;

///< Robosense原始点及点云类
struct EIGEN_ALIGN16 RobosensePoint {
    PCL_ADD_POINT4D;  ///< 三维点（齐次形式）
    float intensity;  ///< 强度
    double timestamp; ///< Robosense：绝对时间戳（s）
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
using RobosensePointCloud = pcl::PointCloud<RobosensePoint>;

namespace point_cloud_converter {

/**
 * @brief 检测雷达类型
 * @param msg 点云信息
 * @return 雷达类型
 */
std::string detectLidarType(const sensor_msgs::msg::PointCloud2 &msg);

/**
 * @brief 将ouster点云格式转换为内部原始点云格式
 * @param src ouster点云格式数据
 * @return 内部原始点云格式数据
 * @warning 返回值的包头时间戳单位为ns，与pcl官方定义不同，使用toROSMsg前需要注意时间戳问题
 */
RawPointCloud::Ptr convert(OusterPointCloud &src);

/**
 * @brief 将velodyne点云格式转换为内部原始点云格式
 * @param src velodyne点云格式数据
 * @return 内部原始点云格式数据
 * @warning 返回值的包头时间戳单位为ns，与pcl官方定义不同，使用toROSMsg前需要注意时间戳问题
 */
RawPointCloud::Ptr convert(VelodynePointCloud &src);

/**
 * @brief 将livox点云格式转换为内部原始点云格式
 * @param src livox点云格式数据
 * @return 内部原始点云格式数据
 * @warning 返回值的包头时间戳单位为ns，与pcl官方定义不同，使用toROSMsg前需要注意时间戳问题
 */
RawPointCloud::Ptr convert(LivoxPointCloud &src);

/**
 * @bried 将Robosense点云格式转换为内部原始点云格式
 * @param src Robosense点云格式数据
 * @return 内部原始点云格式数据
 * @warning 返回值的包头时间戳单位为ns，与pcl官方定义不同，使用toROSMsg前需要注意时间戳问题
 */
RawPointCloud::Ptr convert(RobosensePointCloud &src);

} // namespace point_cloud_converter
} // namespace lvins

// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(lvins::OusterPoint,
                                 (float, x, x)
                                 (float, y, y)
                                 (float, z, z)
                                 (float, intensity, intensity)
                                 (uint32_t, t, t))

POINT_CLOUD_REGISTER_POINT_STRUCT(lvins::VelodynePoint,
                                 (float, x, x)
                                 (float, y, y)
                                 (float, z, z)
                                 (float, intensity, intensity)
                                 (float, time, time))

POINT_CLOUD_REGISTER_POINT_STRUCT(lvins::RobosensePoint,
                                 (float, x, x)
                                 (float, y, y)
                                 (float, z, z)
                                 (float, intensity, intensity)
                                 (double, timestamp, timestamp))
// clang-format on