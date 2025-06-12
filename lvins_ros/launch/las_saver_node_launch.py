"""
@file las_saver_node_launch.py
@brief 启动LAS文件保存节点的启动文件
@details 该文件用于启动一个ROS2节点，该节点负责将点云数据保存为LAS格式的文件
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    使用可配置的参数启动LAS文件保存节点。
    """
    las_save_path_arg = DeclareLaunchArgument(
        "las_save_path",
        default_value="/home/ubuntu/logs",
        description="Path to save the LAS files.",
    )
    las_prefix_arg = DeclareLaunchArgument(
        "las_prefix",
        default_value="map",
        description="Prefix for the saved LAS files.",
    )
    las_point_cloud_topics_arg = DeclareLaunchArgument(
        "las_point_cloud_topics",
        default_value='["/odometry_node/lidar0/point_cloud"]',
        description="List of point cloud topics for las saver to subscribe to.",
    )

    las_saver_node = Node(
        package="lvins_ros",
        executable="las_saver_node.py",
        name="las_saver_node",
        parameters=[
            {
                "save_path": LaunchConfiguration("las_save_path"),
                "las_prefix": LaunchConfiguration("las_prefix"),
                "point_cloud_topics": LaunchConfiguration("las_point_cloud_topics"),
            }
        ],
    )

    return LaunchDescription(
        [
            las_save_path_arg,
            las_prefix_arg,
            las_point_cloud_topics_arg,
            las_saver_node,
        ]
    )
