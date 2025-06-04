#pragma once

#include "lvins_icp/point_cloud.h"

#include <sensor_msgs/msg/point_cloud2.hpp>

namespace lvins::point_cloud_converter {

/**
 * @brief 检测雷达类型
 * @param msg 点云信息
 * @return 雷达类型
 */
std::string detectLidarType(const sensor_msgs::msg::PointCloud2 &msg);

/**
 * @brief 将ROS点云格式数据转换为内部点云格式
 * @param msg ROS点云格式数据
 * @param lidar_type 雷达类型
 * @return 内部点云格式数据
 */
PointCloud::Ptr convert(const sensor_msgs::msg::PointCloud2 &msg, const std::string &lidar_type);

/**
 * @brief 将ROS点云格式数据转换为内部点云格式
 * @param msg ROS点云格式数据
 * @param lidar_type 雷达类型
 * @return 内部点云格式数据
 */
PointCloud::Ptr convert(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg, const std::string &lidar_type);

/**
 * @brief 将内部点云格式数据转换为ROS点云格式
 * @param timestamp 时间戳
 * @param frame_id 点云坐标系id
 * @param point_cloud 内部点云格式数据
 * @return ROS点云格式数据
 */
sensor_msgs::msg::PointCloud2::SharedPtr convert(int64_t timestamp, const std::string &frame_id,
                                                 const PointCloud &point_cloud);

/**
 * @brief 将内部点云格式数据转换为ROS点云格式
 * @param timestamp 时间戳
 * @param frame_id 点云坐标系id
 * @param point_cloud 内部点云格式数据
 * @return ROS点云格式数据
 */
sensor_msgs::msg::PointCloud2::SharedPtr convert(int64_t timestamp, const std::string &frame_id,
                                                 const PointCloud::ConstPtr &point_cloud);

} // namespace lvins::point_cloud_converter