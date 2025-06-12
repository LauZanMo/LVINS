#include "lvins_ros/data_recorder_node.h"
#include "lvins_common/logger.h"
#include "lvins_common/path_helper.h"

using namespace lvins;

DataRecorder::DataRecorder(const std::string &program_name) : Node(program_name, program_name) {
    // 获取参数
    declare_parameter<std::string>("save_path", "");
    declare_parameter<std::string>("bag_prefix", "raw_data");
    declare_parameter<std::string>("imu_topic", "/imu0/data_raw");
    declare_parameter<std::string>("gnss_topic", "/gnss0/fix");
    point_cloud_topics_ = {"/lidar0/point_cloud_raw"};
    declare_parameter<std::vector<std::string>>("point_cloud_topics", point_cloud_topics_);
    image_topics_ = {"/camera0/image_raw"};
    declare_parameter<std::vector<std::string>>("image_topics", image_topics_);

    save_path_          = get_parameter("save_path").as_string();
    bag_prefix_         = get_parameter("bag_prefix").as_string();
    imu_topic_          = get_parameter("imu_topic").as_string();
    gnss_topic_         = get_parameter("gnss_topic").as_string();
    point_cloud_topics_ = get_parameter("point_cloud_topics").as_string_array();
    image_topics_       = get_parameter("image_topics").as_string_array();

    // 检查参数并打印
    if (save_path_.empty()) {
        RCLCPP_FATAL(get_logger(), "Save path should not be empty!");
    }
    if (bag_prefix_.empty()) {
        RCLCPP_FATAL(get_logger(), "Bag prefix should not be empty!");
    }
    save_path_ = path_helper::completePath(save_path_);

    RCLCPP_INFO(get_logger(), "Data save path: %s", save_path_.c_str());
    RCLCPP_INFO(get_logger(), "Data bag prefix: %s", bag_prefix_.c_str());
    RCLCPP_INFO(get_logger(), "IMU topic: %s", imu_topic_.c_str());
    RCLCPP_INFO(get_logger(), "GNSS topic: %s", gnss_topic_.c_str());
    const auto point_cloud_topics_str = LVINS_FORMAT("[{}]", LVINS_JOIN(point_cloud_topics_, ", "));
    RCLCPP_INFO(get_logger(), "Point cloud topics: %s", point_cloud_topics_str.c_str());
    const auto image_topics_str = LVINS_FORMAT("[{}]", LVINS_JOIN(image_topics_, ", "));
    RCLCPP_INFO(get_logger(), "Image topics: %s", image_topics_str.c_str());

    // 初始化Logger
    Logger::initialize(true, save_path_, program_name);
    LVINS_INFO("Starting Data recorder...");

    // 初始化写入器
    writer_ = std::make_unique<rosbag2_cpp::Writer>();

    // 设置服务质量（QoS）
    rclcpp::QoS qos(rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST, 1000));
    qos.reliability(rclcpp::ReliabilityPolicy::Reliable);
    qos.durability(rclcpp::DurabilityPolicy::SystemDefault);

    { // 创建IMU订阅
        using ImuCallback = std::function<void(const sensor_msgs::msg::Imu::ConstSharedPtr &)>;
        ImuCallback cb    = [this](auto &&msg) {
            this->imuCallback(imu_topic_, std::forward<decltype(msg)>(msg));
        };
        imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(imu_topic_, qos, cb);
    }

    { // 创建GNSS订阅
        using GnssCallback = std::function<void(const sensor_msgs::msg::NavSatFix::ConstSharedPtr &)>;
        GnssCallback cb    = [this](auto &&msg) {
            this->gnssCallback(gnss_topic_, std::forward<decltype(msg)>(msg));
        };
        gnss_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(gnss_topic_, qos, cb);
    }

    { // 创建点云订阅
        using PointCloudCallback = std::function<void(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &)>;
        point_cloud_subs_.resize(point_cloud_topics_.size());
        for (size_t i = 0; i < point_cloud_topics_.size(); ++i) {
            PointCloudCallback cb = [this, i](auto &&msg) {
                this->pointCloudCallback(point_cloud_topics_[i], std::forward<decltype(msg)>(msg));
            };
            point_cloud_subs_[i] = create_subscription<sensor_msgs::msg::PointCloud2>(point_cloud_topics_[i], qos, cb);
        }
    }

    { // 创建图像订阅
        using ImageCallback = std::function<void(const sensor_msgs::msg::Image::ConstSharedPtr &)>;
        image_subs_.resize(image_topics_.size());
        for (size_t i = 0; i < image_topics_.size(); ++i) {
            ImageCallback cb = [this, i](auto &&msg) {
                this->imageCallback(image_topics_[i], std::forward<decltype(msg)>(msg));
            };
            image_subs_[i] = create_subscription<sensor_msgs::msg::Image>(image_topics_[i], qos, cb);
        }
    }

    { // 初始化服务
        using TriggerCallback = std::function<void(const std_srvs::srv::Trigger::Request::ConstSharedPtr &,
                                                   const std_srvs::srv::Trigger::Response::SharedPtr &)>;

        // 开始录制服务
        TriggerCallback start_cb = [this](auto &&request, auto &&response) {
            this->startRecordingCallback(std::forward<decltype(request)>(request),
                                         std::forward<decltype(response)>(response));
        };
        start_srv_ = create_service<std_srvs::srv::Trigger>("~/start", start_cb);

        // 结束录制服务
        TriggerCallback stop_cb = [this](auto &&request, auto &&response) {
            this->stopRecordingCallback(std::forward<decltype(request)>(request),
                                        std::forward<decltype(response)>(response));
        };
        stop_srv_ = create_service<std_srvs::srv::Trigger>("~/stop", stop_cb);
    }

    LVINS_INFO("Data recorder started!");
}

DataRecorder::~DataRecorder() {
    LVINS_INFO("Closing data recorder node...");

    // 检测是否正在录制
    if (recording_) {
        // 关闭rosbag
        writer_->close();
        LVINS_INFO("Data record closed!");
    }

    LVINS_INFO("Data recorder node closed!");

    // 关闭Logger
    Logger::shutdown();
}

void DataRecorder::imuCallback(const std::string &topic, const sensor_msgs::msg::Imu::ConstSharedPtr &msg) {
    if (recording_) {
        writer_->write(*msg, topic, msg->header.stamp);
    }
}

void DataRecorder::gnssCallback(const std::string &topic, const sensor_msgs::msg::NavSatFix::ConstSharedPtr &msg) {
    if (recording_) {
        writer_->write(*msg, topic, msg->header.stamp);
    }
}

void DataRecorder::pointCloudCallback(const std::string &topic,
                                      const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg) {
    if (recording_) {
        writer_->write(*msg, topic, msg->header.stamp);
    }
}

void DataRecorder::imageCallback(const std::string &topic, const sensor_msgs::msg::Image::ConstSharedPtr &msg) {
    if (recording_) {
        writer_->write(*msg, topic, msg->header.stamp);
    }
}

void DataRecorder::startRecordingCallback(const std_srvs::srv::Trigger::Request::ConstSharedPtr & /*request*/,
                                          const std_srvs::srv::Trigger::Response::SharedPtr &response) {
    // 检测是否正在录制
    if (recording_) {
        LVINS_WARN("Fail to start recording since it is already started.");
        response->success = false;
        response->message = "Recording is already started.";
        return;
    }

    // 生成文件名
    save_path_     = get_parameter("save_path").as_string();
    bag_prefix_    = get_parameter("bag_prefix").as_string();
    const auto uri = LVINS_FORMAT("{}/{}{}", save_path_, bag_prefix_, record_index_++);

    // 创建rosbag
    writer_->open(uri);
    LVINS_INFO("Create raw data record: {}", uri);

    // 设置标志位
    recording_ = true;

    // 返回响应
    response->success = true;
    response->message = "Start recording successfully.";
}

void DataRecorder::stopRecordingCallback(const std_srvs::srv::Trigger::Request::ConstSharedPtr & /*request*/,
                                         const std_srvs::srv::Trigger::Response::SharedPtr &response) {
    // 检测是否还未录制
    if (!recording_) {
        LVINS_WARN("Fail to stop recording since it is not started.");
        response->success = false;
        response->message = "Recording is not started.";
        return;
    }

    // 关闭rosbag
    writer_->close();
    LVINS_INFO("Data record closed!");

    // 设置标志位
    recording_ = false;

    // 返回响应
    response->success = true;
    response->message = "Stop recording successfully.";
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    const auto program_name = path_helper::getFileName(argv[0]);
    rclcpp::spin(std::make_shared<DataRecorder>(program_name));
    rclcpp::shutdown();

    return 0;
}
