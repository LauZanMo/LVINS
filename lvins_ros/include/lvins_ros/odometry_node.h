#pragma once

#include "lvins_ros/message_synchronizer.h"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/trigger.hpp>

class OdometryNode final : public rclcpp::Node {
public:
    /**
     * @brief 构造函数
     * @param program_name 程序名称
     */
    explicit OdometryNode(const std::string &program_name);

    /**
     * @brief 析构函数
     */
    ~OdometryNode() override;

    /**
     * @brief IMU信息接收回调函数
     * @param msg IMU信息
     */
    void imuCallback(const sensor_msgs::msg::Imu::ConstSharedPtr &msg);

    /**
     * @brief 点云信息接收回调函数
     * @param idx 点云所属传感器索引
     * @param msg 点云信息
     */
    void pointCloudCallback(size_t idx, const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg);

    /**
     * @brief 图像信息接收回调函数
     * @param idx 图像所属传感器索引
     * @param msg 图像信息
     */
    void imageCallback(size_t idx, const sensor_msgs::msg::Image::ConstSharedPtr &msg);

private:
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;                               ///< IMU信息订阅者
    std::vector<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr> point_cloud_subs_; ///< 点云信息订阅者
    rclcpp::CallbackGroup::SharedPtr point_cloud_callback_group_;                                  ///< 点云信息回调组
    std::vector<rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr> image_subs_;             ///< 图像信息订阅者
    rclcpp::CallbackGroup::SharedPtr image_callback_group_;                                        ///< 图像信息回调组
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_srv_;                                 ///< 重置服务

    lvins::MessageSynchronizer<sensor_msgs::msg::PointCloud2>::uPtr point_cloud_sync_; ///< 点云同步器
    lvins::MessageSynchronizer<sensor_msgs::msg::Image>::uPtr image_sync_;             ///< 图像同步器

    std::vector<std::string> lidar_types_; ///< 雷达类型
};
