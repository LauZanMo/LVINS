#pragma once

#include "lvins_odometry/estimator.h"
#include "lvins_ros/utils/message_synchronizer.h"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/trigger.hpp>

/**
 * @brief 里程计节点类
 * @details 该类用于接收、同步来自ROS的IMU、点云和图像数据，并将其输入到估计器中进行状态估计，同时负责提供估计器的服务
 */
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
     * @brief GNSS信息接收回调函数
     * @param msg GNSS信息
     */
    void gnssCallback(const sensor_msgs::msg::NavSatFix::ConstSharedPtr &msg);

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
    lvins::Estimator::Ptr estimator_; ///< 估计器

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;                               ///< IMU信息订阅者
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gnss_sub_;                        ///< GNSS信息订阅者
    std::vector<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr> point_cloud_subs_; ///< 点云信息订阅者
    rclcpp::CallbackGroup::SharedPtr point_cloud_callback_group_;                                  ///< 点云信息回调组
    std::vector<rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr> image_subs_;             ///< 图像信息订阅者
    rclcpp::CallbackGroup::SharedPtr image_callback_group_;                                        ///< 图像信息回调组
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_srv_;                                 ///< 重置服务

    lvins::MessageSynchronizer<sensor_msgs::msg::PointCloud2>::Ptr point_cloud_sync_; ///< 点云同步器
    lvins::MessageSynchronizer<sensor_msgs::msg::Image>::Ptr image_sync_;             ///< 图像同步器

    std::vector<std::string> lidar_types_; ///< 雷达类型
};
