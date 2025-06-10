"""
@file odometry_node_launch.py
@brief 启动里程计节点的启动文件
@details 该文件用于启动一个ROS2节点，该节点负责处理IMU和点云数据，并进行里程计计算
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    使用可配置的参数启动里程计节点
    """
    odometry_log_path_arg = DeclareLaunchArgument(
        "odometry_log_path",
        default_value="/home/ubuntu/logs",
        description="Path to save odometry logs.",
    )
    odometry_config_file_arg = DeclareLaunchArgument(
        "odometry_config_file",
        default_value="config/odometry.yaml",
        description="Path to the odometry configuration file.",
    )
    odometry_imu_topic_arg = DeclareLaunchArgument(
        "odometry_imu_topic",
        default_value="/imu0/data_raw",
        description="IMU topic for odometry to subscribe to.",
    )
    odometry_gnss_topic_arg = DeclareLaunchArgument(
        "odometry_gnss_topic",
        default_value="/gnss0/fix",
        description="GNSS topic for odometry to subscribe to.",
    )
    odometry_point_cloud_topics_arg = DeclareLaunchArgument(
        "odometry_point_cloud_topics",
        default_value='["/lidar0/point_cloud_raw"]',
        description="List of point cloud topics for odometry to subscribe to.",
    )
    odometry_image_topics_arg = DeclareLaunchArgument(
        "odometry_image_topics",
        default_value='["/camera0/image_raw"]',
        description="List of image topics for odometry to subscribe to.",
    )
    odometry_reset_topic_arg = DeclareLaunchArgument(
        "odometry_reset_topic",
        default_value="/reset",
        description="Service topic to reset the odometry.",
    )

    odometry_node = Node(
        package="lvins_ros",
        executable="odometry_node",
        name="odometry_node",
        output="screen",
        parameters=[
            {
                "log_path": LaunchConfiguration("odometry_log_path"),
                "config_file": LaunchConfiguration("odometry_config_file"),
                "imu_topic": LaunchConfiguration("odometry_imu_topic"),
                "gnss_topic": LaunchConfiguration("odometry_gnss_topic"),
                "point_cloud_topics": LaunchConfiguration(
                    "odometry_point_cloud_topics"
                ),
                "image_topics": LaunchConfiguration("odometry_image_topics"),
                "reset_topic": LaunchConfiguration("odometry_reset_topic"),
            }
        ],
    )

    return LaunchDescription(
        [
            odometry_log_path_arg,
            odometry_config_file_arg,
            odometry_imu_topic_arg,
            odometry_gnss_topic_arg,
            odometry_point_cloud_topics_arg,
            odometry_image_topics_arg,
            odometry_reset_topic_arg,
            odometry_node,
        ]
    )
