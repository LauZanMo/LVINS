"""
@file data_recorder_node_launch.py
@brief 启动数据记录器节点的启动文件
@details 该文件用于启动一个ROS2节点，该节点负责记录原始数据（IMU、GNSS、点云、图像）到bag文件中
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    使用可配置的参数启动数据记录器节点。
    """
    record_save_path_arg = DeclareLaunchArgument(
        "record_save_path",
        default_value="/home/ubuntu/logs",
        description="Path to save the recorded data.",
    )
    bag_prefix_arg = DeclareLaunchArgument(
        "bag_prefix",
        default_value="raw_data",
        description="Prefix for the recorded bag files.",
    )
    raw_imu_topic_arg = DeclareLaunchArgument(
        "raw_imu_topic",
        default_value="/imu0/data_raw",
        description="IMU topic for data recorder to subscribe to.",
    )
    raw_gnss_topic_arg = DeclareLaunchArgument(
        "raw_gnss_topic",
        default_value="/gnss0/fix",
        description="GNSS topic for data recorder to subscribe to.",
    )
    raw_point_cloud_topics_arg = DeclareLaunchArgument(
        "raw_point_cloud_topics",
        default_value='["/lidar0/point_cloud_raw"]',
        description="List of point cloud topics for data recorder to subscribe to.",
    )
    raw_image_topics_arg = DeclareLaunchArgument(
        "raw_image_topics",
        default_value='["/camera0/image_raw"]',
        description="List of image topics for data recorder to subscribe to.",
    )

    data_recorder_node = Node(
        package="lvins_ros",
        executable="data_recorder_node",
        name="data_recorder_node",
        output="screen",
        parameters=[
            {
                "save_path": LaunchConfiguration("record_save_path"),
                "bag_prefix": LaunchConfiguration("bag_prefix"),
                "imu_topic": LaunchConfiguration("raw_imu_topic"),
                "gnss_topic": LaunchConfiguration("raw_gnss_topic"),
                "point_cloud_topics": LaunchConfiguration("raw_point_cloud_topics"),
                "image_topics": LaunchConfiguration("raw_image_topics"),
            }
        ],
    )

    return LaunchDescription(
        [
            record_save_path_arg,
            bag_prefix_arg,
            raw_imu_topic_arg,
            raw_gnss_topic_arg,
            raw_point_cloud_topics_arg,
            raw_image_topics_arg,
            data_recorder_node,
        ]
    )
