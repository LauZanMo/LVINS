#pragma once

#include "lvins_common/yaml/yaml_serialization.h"
#include "lvins_odometry/drawer_base.h"

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/u_int64.hpp>
#include <tf2_ros/transform_broadcaster.h>

namespace lvins {

/**
 * @brief Rviz绘制器类
 * @details 该类用于在Rviz中可视化里程计系统中的各种数据
 */
class DrawerRviz final : public DrawerBase {
public:
    /**
     * @brief 构造函数
     * @param config YAML配置节点
     * @param node ROS节点，用于发布消息
     */
    DrawerRviz(const YAML::Node &config, rclcpp::Node &node);

private:
    /**
     * @brief 绘制雷达帧束
     * @param timestamp 雷达帧束时间戳
     * @param bundle 雷达帧束
     */
    void drawLidarFrameBundle(int64_t timestamp, const LidarFrameBundle::sPtr &bundle) override;

    /**
     * @brief 发布重置次数
     * @param times 重置次数
     */
    void publishResetTimes(size_t times) override;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr current_nav_state_pub_;
    std::vector<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr> frame_point_cloud_pubs_;
    rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr reset_times_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    std::vector<std::string> lidar_frame_ids_;
    std::string imu_frame_id_;
    std::string map_frame_id_;
};

} // namespace lvins
