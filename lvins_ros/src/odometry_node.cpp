#include "lvins_ros/odometry_node.h"
#include "lvins_common/logger.h"
#include "lvins_common/path_helper.h"
#include "lvins_common/sensor/imu.h"
#include "lvins_ros/impl/drawer_rviz.h"
#include "lvins_ros/utils/point_cloud_converter.h"

#include <cv_bridge/cv_bridge.h>
#include <pcl_conversions/pcl_conversions.h>

#define BACKWARD_HAS_DW 1
#include <backward.hpp>
backward::SignalHandling sh;

using namespace lvins;

OdometryNode::OdometryNode(const std::string &program_name) : Node(program_name, program_name) {
    // 获取参数
    declare_parameter<std::string>("log_path", "");
    declare_parameter<std::string>("config_file", "");
    declare_parameter<std::string>("imu_topic", "/imu0/data_raw");
    std::vector<std::string> point_cloud_topics = {"/lidar0/point_cloud_raw"};
    declare_parameter<std::vector<std::string>>("point_cloud_topics", point_cloud_topics);
    std::vector<std::string> image_topics = {"/camera0/image_raw"};
    declare_parameter<std::vector<std::string>>("image_topics", image_topics);
    declare_parameter<std::string>("reset_topic", "/reset");

    auto log_path          = get_parameter("log_path").as_string();
    auto config_file       = get_parameter("config_file").as_string();
    const auto imu_topic   = get_parameter("imu_topic").as_string();
    point_cloud_topics     = get_parameter("point_cloud_topics").as_string_array();
    image_topics           = get_parameter("image_topics").as_string_array();
    const auto reset_topic = get_parameter("reset_topic").as_string();

    // 检查参数并打印
    if (log_path.empty()) {
        RCLCPP_FATAL(get_logger(), "Log path should not be empty!");
    }
    if (config_file.empty()) {
        RCLCPP_FATAL(get_logger(), "Configuration file should not be empty!");
    }
    log_path    = path_helper::completePath(log_path);
    config_file = path_helper::completePath(config_file);

    RCLCPP_INFO(get_logger(), "Log path: %s", log_path.c_str());
    RCLCPP_INFO(get_logger(), "Configuration file: %s", config_file.c_str());
    RCLCPP_INFO(get_logger(), "IMU topic: %s", imu_topic.c_str());
    const auto point_cloud_topics_str = LVINS_FORMAT("[{}]", LVINS_JOIN(point_cloud_topics, ", "));
    RCLCPP_INFO(get_logger(), "Point cloud topics: %s", point_cloud_topics_str.c_str());
    const auto image_topics_str = LVINS_FORMAT("[{}]", LVINS_JOIN(image_topics, ", "));
    RCLCPP_INFO(get_logger(), "Image topics: %s", image_topics_str.c_str());
    RCLCPP_INFO(get_logger(), "Reset topic: %s", reset_topic.c_str());

    // 初始化Logger
    Logger::initialize(true, log_path, program_name);

    // 加载配置
    LVINS_INFO("Starting odometry node...");
    const auto config = YAML::load(config_file);

    // 初始化估计器
    DrawerBase::Ptr drawer = std::make_unique<DrawerRviz>(config["drawer"], *this);
    estimator_             = std::make_unique<Estimator>(config, std::move(drawer));

    // 设置服务质量（QoS）
    rclcpp::QoS qos(rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST, 1000));
    qos.reliability(rclcpp::ReliabilityPolicy::Reliable);
    qos.durability(rclcpp::DurabilityPolicy::SystemDefault);

    { // 创建IMU订阅
        using ImuCallback = std::function<void(const sensor_msgs::msg::Imu::ConstSharedPtr &)>;
        ImuCallback cb    = [this](auto &&msg) {
            this->imuCallback(std::forward<decltype(msg)>(msg));
        };
        imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(imu_topic, qos, cb);
    }

    { // 创建点云订阅
        point_cloud_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        rclcpp::SubscriptionOptions sub_option;
        sub_option.callback_group = point_cloud_callback_group_;

        using PointCloudCallback = std::function<void(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &)>;
        point_cloud_subs_.resize(point_cloud_topics.size());
        for (size_t i = 0; i < point_cloud_topics.size(); ++i) {
            PointCloudCallback cb = [this, i](auto &&msg) {
                this->pointCloudCallback(i, std::forward<decltype(msg)>(msg));
            };
            point_cloud_subs_[i] =
                    create_subscription<sensor_msgs::msg::PointCloud2>(point_cloud_topics[i], qos, cb, sub_option);
        }

        point_cloud_sync_ =
                std::make_unique<MessageSynchronizer<sensor_msgs::msg::PointCloud2>>(point_cloud_topics.size());
        lidar_types_.resize(point_cloud_topics.size());
    }

    { // 创建图像订阅
        image_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        rclcpp::SubscriptionOptions sub_option;
        sub_option.callback_group = image_callback_group_;

        using ImageCallback = std::function<void(const sensor_msgs::msg::Image::ConstSharedPtr &)>;
        image_subs_.resize(image_topics.size());
        for (size_t i = 0; i < image_topics.size(); ++i) {
            ImageCallback cb = [this, i](auto &&msg) {
                this->imageCallback(i, std::forward<decltype(msg)>(msg));
            };
            image_subs_[i] = create_subscription<sensor_msgs::msg::Image>(image_topics[i], qos, cb, sub_option);
        }

        image_sync_ = std::make_unique<MessageSynchronizer<sensor_msgs::msg::Image>>(image_topics.size());
    }

    { // 创建重置服务
        using ResetCallback = std::function<void(const std_srvs::srv::Trigger::Request::ConstSharedPtr &,
                                                 const std_srvs::srv::Trigger::Response::SharedPtr &)>;
        ResetCallback cb    = [this]([[maybe_unused]] auto &&request, auto &&response) {
            estimator_->reset();
            response->success = true;
            response->message = "Odometry node reset!";
        };
        reset_srv_ = create_service<std_srvs::srv::Trigger>(reset_topic, cb, qos.get_rmw_qos_profile());
    }

    LVINS_INFO("Odometry node started!");
}

OdometryNode::~OdometryNode() {
    LVINS_INFO("Closing odometry node...");

    LVINS_INFO("Odometry node closed!");

    // 关闭Logger
    Logger::shutdown();
}

void OdometryNode::imuCallback(const sensor_msgs::msg::Imu::ConstSharedPtr &msg) {
    // 将信息转换为系统输入类型
    Imu imu;
    imu.timestamp = rclcpp::Time(msg->header.stamp).nanoseconds();
    imu.gyr       = {LVINS_FLOAT(msg->angular_velocity.x), LVINS_FLOAT(msg->angular_velocity.y),
                     LVINS_FLOAT(msg->angular_velocity.z)};
    imu.acc       = {LVINS_FLOAT(msg->linear_acceleration.x), LVINS_FLOAT(msg->linear_acceleration.y),
                     LVINS_FLOAT(msg->linear_acceleration.z)};

    // 输入系统
    estimator_->addImu(imu);
}

void OdometryNode::pointCloudCallback(size_t idx, const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg) {
    if (point_cloud_sync_->push(idx, msg)) {
        const auto &point_cloud_msgs = point_cloud_sync_->getSyncMessages();

        // 转换为内部信息
        std::vector<RawPointCloud::Ptr> raw_point_clouds;
        for (size_t i = 0; i < point_cloud_msgs.size(); ++i) {
            if (lidar_types_[i].empty()) {
                lidar_types_[i] = point_cloud_converter::detectLidarType(*point_cloud_msgs[i]);
                LVINS_INFO("Lidar #{} type detected: {}", i, lidar_types_[i]);
            }

            RawPointCloud::Ptr raw_point_cloud;
            if (lidar_types_[i] == "ouster") {
                const auto os_point_cloud = pcl::make_shared<OusterPointCloud>();
                pcl::fromROSMsg(*point_cloud_msgs[i], *os_point_cloud);
                raw_point_cloud = point_cloud_converter::convert(*os_point_cloud);
            } else if (lidar_types_[i] == "velodyne") {
                const auto vld_point_cloud = pcl::make_shared<VelodynePointCloud>();
                pcl::fromROSMsg(*point_cloud_msgs[i], *vld_point_cloud);
                raw_point_cloud = point_cloud_converter::convert(*vld_point_cloud);
            } else if (lidar_types_[i] == "livox") {
                const auto lv_point_cloud = pcl::make_shared<LivoxPointCloud>();
                pcl::fromROSMsg(*point_cloud_msgs[i], *lv_point_cloud);
                raw_point_cloud = point_cloud_converter::convert(*lv_point_cloud);
            } else if (lidar_types_[i] == "robosense") {
                const auto rs_point_cloud = pcl::make_shared<RobosensePointCloud>();
                pcl::fromROSMsg(*point_cloud_msgs[i], *rs_point_cloud);
                raw_point_cloud = point_cloud_converter::convert(*rs_point_cloud);
            } else {
                return;
            }

            raw_point_clouds.push_back(std::move(raw_point_cloud));
        }

        // 输入系统
        const auto timestamp = rclcpp::Time(point_cloud_msgs[0]->header.stamp).nanoseconds();
        estimator_->addPointClouds(timestamp, raw_point_clouds);
    }
}

void OdometryNode::imageCallback(size_t idx, const sensor_msgs::msg::Image::ConstSharedPtr &msg) {
    if (image_sync_->push(idx, msg)) {
        const auto &image_msgs = image_sync_->getSyncMessages();

        // 转换为内部信息
        std::vector<cv::Mat> raw_images;
        for (const auto &image_msg: image_msgs) {
            raw_images.push_back(cv_bridge::toCvCopy(image_msg)->image);
        }

        // 输入系统
        const auto timestamp = rclcpp::Time(image_msgs[0]->header.stamp).nanoseconds();
        estimator_->addImages(timestamp, raw_images);
    }
}

int main(int argc, char **argv) {
    // 启动ROS节点
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor;
    const auto program_name = path_helper::getFileName(argv[0]);
    const auto node         = std::make_shared<OdometryNode>(program_name);
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();

    return 0;
}
