#include "lvins_ros/impl/drawer_rviz.h"

#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

namespace lvins {

DrawerRviz::DrawerRviz(const YAML::Node &config, rclcpp::Node &node) {
    // 获取话题参数
    const auto topic_config             = config["topic"];
    const auto current_nav_state_topic  = YAML::get<std::string>(topic_config, "current_nav_state");
    const auto frame_point_cloud_topics = YAML::get<std::vector<std::string>>(topic_config, "frame_point_clouds");
    const auto frame_compress_point_cloud_topics =
            YAML::get<std::vector<std::string>>(topic_config, "frame_compress_point_clouds");
    const auto submap_topic = YAML::get<std::string>(topic_config, "submap");
    const auto reset_topic  = YAML::get<std::string>(topic_config, "reset");

    // 获取坐标系id参数
    const auto frame_id_config = config["frame_id"];
    lidar_frame_ids_           = YAML::get<std::vector<std::string>>(frame_id_config, "lidars");
    imu_frame_id_              = YAML::get<std::string>(frame_id_config, "imu");
    map_frame_id_              = YAML::get<std::string>(frame_id_config, "map");

    // 执行必要的检查
    LVINS_CHECK(frame_point_cloud_topics.size() == lidar_frame_ids_.size(),
                "The size of frame point clouds and lidars should be the same!");
    LVINS_CHECK(frame_compress_point_cloud_topics.size() == lidar_frame_ids_.size(),
                "The size of frame compress point clouds and lidars should be the same!");

    // 设置服务质量（QoS）
    rclcpp::QoS qos(rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST, 100));
    qos.reliability(rclcpp::ReliabilityPolicy::SystemDefault);
    qos.durability(rclcpp::DurabilityPolicy::SystemDefault);

    // 创建发布者
    current_nav_state_pub_ = node.create_publisher<nav_msgs::msg::Odometry>(current_nav_state_topic, qos);
    for (const auto &topic: frame_point_cloud_topics) {
        frame_point_cloud_pubs_.push_back(node.create_publisher<sensor_msgs::msg::PointCloud2>(topic, qos));
    }
    for (const auto &topic: frame_compress_point_cloud_topics) {
        frame_compress_point_cloud_pubs_.push_back(node.create_publisher<sensor_msgs::msg::PointCloud2>(topic, qos));
    }
    reset_times_pub_ = node.create_publisher<std_msgs::msg::UInt64>(reset_topic, qos);
    tf_broadcaster_  = std::make_unique<tf2_ros::TransformBroadcaster>(node);
}

void DrawerRviz::drawLidarFrameBundle(int64_t timestamp, const LidarFrameBundle::Ptr &bundle) {
    LVINS_CHECK(frame_point_cloud_pubs_.size() == bundle->size(),
                "Publishers and frame bundle should have the same size!");

    // 发布导航状态
    rclcpp::Time ros_timestamp(timestamp);
    const auto &state = bundle->state();
    if (current_nav_state_pub_->get_subscription_count() != 0) {
        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp            = ros_timestamp;
        odom_msg.header.frame_id         = map_frame_id_;
        odom_msg.child_frame_id          = imu_frame_id_;
        odom_msg.pose.pose.position.x    = state.T.translation()[0];
        odom_msg.pose.pose.position.y    = state.T.translation()[1];
        odom_msg.pose.pose.position.z    = state.T.translation()[2];
        odom_msg.pose.pose.orientation.x = state.T.unit_quaternion().x();
        odom_msg.pose.pose.orientation.y = state.T.unit_quaternion().y();
        odom_msg.pose.pose.orientation.z = state.T.unit_quaternion().z();
        odom_msg.pose.pose.orientation.w = state.T.unit_quaternion().w();
        odom_msg.twist.twist.linear.x    = state.vel[0];
        odom_msg.twist.twist.linear.y    = state.vel[1];
        odom_msg.twist.twist.linear.z    = state.vel[2];
        current_nav_state_pub_->publish(odom_msg);
    }

    // 发布TF
    geometry_msgs::msg::TransformStamped trans_msg;
    trans_msg.header.stamp            = rclcpp::Time(timestamp);
    trans_msg.header.frame_id         = map_frame_id_;
    trans_msg.child_frame_id          = imu_frame_id_;
    trans_msg.transform.translation.x = state.T.translation()[0];
    trans_msg.transform.translation.y = state.T.translation()[1];
    trans_msg.transform.translation.z = state.T.translation()[2];
    trans_msg.transform.rotation.x    = state.T.unit_quaternion().x();
    trans_msg.transform.rotation.y    = state.T.unit_quaternion().y();
    trans_msg.transform.rotation.z    = state.T.unit_quaternion().z();
    trans_msg.transform.rotation.w    = state.T.unit_quaternion().w();
    tf_broadcaster_->sendTransform(trans_msg);

    for (size_t i = 0; i < bundle->size(); ++i) {
        const auto &T_bs                  = bundle->frame(i).Tbs();
        trans_msg.header.frame_id         = imu_frame_id_;
        trans_msg.child_frame_id          = lidar_frame_ids_[i];
        trans_msg.transform.translation.x = T_bs.translation()[0];
        trans_msg.transform.translation.y = T_bs.translation()[1];
        trans_msg.transform.translation.z = T_bs.translation()[2];
        trans_msg.transform.rotation.x    = T_bs.unit_quaternion().x();
        trans_msg.transform.rotation.y    = T_bs.unit_quaternion().y();
        trans_msg.transform.rotation.z    = T_bs.unit_quaternion().z();
        trans_msg.transform.rotation.w    = T_bs.unit_quaternion().w();
        tf_broadcaster_->sendTransform(trans_msg);
    }

    // 发布点云
    for (size_t i = 0; i < bundle->size(); ++i) {
        // 原始点云
        if (frame_point_cloud_pubs_[i]->get_subscription_count() != 0) {
            const auto &frame      = bundle->frame(i);
            const auto point_cloud = pcl::make_shared<RawPointCloud>();
            pcl::transformPointCloud(frame.rawPointCloud(), *point_cloud, frame.Twf().matrix());

            sensor_msgs::msg::PointCloud2 msg;
            pcl::toROSMsg(*point_cloud, msg);
            msg.header.stamp    = ros_timestamp;
            msg.header.frame_id = map_frame_id_;
            frame_point_cloud_pubs_[i]->publish(msg);
        }

        // 压缩点云
        if (frame_compress_point_cloud_pubs_[i]->get_subscription_count() != 0) {
            const auto &frame      = bundle->frame(i);
            const auto point_cloud = pcl::make_shared<PointCloud>();
            pcl::transformPointCloud(frame.pointCloud(), *point_cloud, frame.Twf().matrix());

            sensor_msgs::msg::PointCloud2 msg;
            pcl::toROSMsg(*point_cloud, msg);
            msg.header.stamp    = ros_timestamp;
            msg.header.frame_id = map_frame_id_;
            frame_compress_point_cloud_pubs_[i]->publish(msg);
        }
    }
}

void DrawerRviz::publishResetTimes(size_t times) {
    if (reset_times_pub_->get_subscription_count() != 0) {
        std_msgs::msg::UInt64 msg;
        msg.data = times;
        reset_times_pub_->publish(msg);
    }
}

} // namespace lvins
