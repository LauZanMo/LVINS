#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/trigger.hpp>

/**
 * @brief 原始数据录制器节点
 */
class DataRecorderNode final : public rclcpp::Node {
public:
    /**
     * @brief 构造函数
     * @param program_name 程序名称
     */
    explicit DataRecorderNode(const std::string &program_name);

    /**
     * @brief 析构函数
     */
    ~DataRecorderNode() override;

private:
    /**
     * @brief IMU消息回调函数
     * @param topic 话题名
     * @param msg IMU消息
     */
    void imuCallback(const std::string &topic, const sensor_msgs::msg::Imu::ConstSharedPtr &msg);

    /**
     * @brief GNSS消息回调函数
     * @param topic 话题名
     * @param msg GNSS消息
     */
    void gnssCallback(const std::string &topic, const sensor_msgs::msg::NavSatFix::ConstSharedPtr &msg);

    /**
     * @brief 点云消息回调函数
     * @param topic 话题名
     * @param msg 点云消息
     */
    void pointCloudCallback(const std::string &topic, const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg);

    /**
     * @brief 图像消息回调函数
     * @param topic 话题名
     * @param msg 图像消息
     */
    void imageCallback(const std::string &topic, const sensor_msgs::msg::Image::ConstSharedPtr &msg);

    /**
     * @brief 开始录制回调函数
     * @param request 请求（空）
     * @param response 响应
     */
    void startRecordingCallback(const std_srvs::srv::Trigger::Request::ConstSharedPtr &request,
                                const std_srvs::srv::Trigger::Response::SharedPtr &response);

    /**
     * @brief 结束录制回调函数
     * @param request 请求（空）
     * @param response 响应
     */
    void stopRecordingCallback(const std_srvs::srv::Trigger::Request::ConstSharedPtr &request,
                               const std_srvs::srv::Trigger::Response::SharedPtr &response);

    std::unique_ptr<rosbag2_cpp::Writer> writer_;                                                  ///< rosbag2写入器
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;                               ///< IMU消息订阅者
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gnss_sub_;                        ///< GNSS消息订阅者
    std::vector<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr> point_cloud_subs_; ///< 点云消息订阅者
    std::vector<rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr> image_subs_;             ///< 图像消息订阅者
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_srv_;                                 ///< 开始录制服务
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_srv_;                                  ///< 结束录制服务

    std::atomic<bool> recording_{false}; ///< 录制标志位
    size_t record_index_{0};             ///< 录制索引，用于生成唯一的文件名

    std::string save_path_;                       ///< 保存路径
    std::string bag_prefix_;                      ///< rosbag2文件名前缀
    std::string imu_topic_;                       ///< IMU消息话题名
    std::string gnss_topic_;                      ///< GNSS消息话题名
    std::vector<std::string> point_cloud_topics_; ///< 点云消息话题名集合
    std::vector<std::string> image_topics_;       ///< 图像消息话题名集合
};
