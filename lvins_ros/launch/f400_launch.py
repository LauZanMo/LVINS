"""
@file f400_launch.py
@brief 启动f400系统的所有相关节点的启动文件
@details 该启动文件用于启动与f400系统相关的所有节点：SLAM节点、LAS文件保存节点和原始数据记录节点
"""

import os
from datetime import datetime
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    """
    使用可配置的参数启动 小球f400 所有相关的节点
    """
    f400_log_path_arg = DeclareLaunchArgument(
        "f400_log_path",
        default_value="/home/ubuntu/logs",
        description="Path to save f400 logs.",
    )
    f400_imu_topic_arg = DeclareLaunchArgument(
        "f400_imu_topic",
        default_value="/livox/imu",
        description="IMU topic for f400 system to subscribe to.",
    )
    f400_point_cloud_topics_arg = DeclareLaunchArgument(
        "f400_point_cloud_topics",
        default_value='["/livox/lidar"]',
        description="Lidar topic for f400 system to subscribe to.",
    )

    current_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    f400_log_path = PathJoinSubstitution(
        [
            LaunchConfiguration("f400_log_path"),
            current_time,
        ]
    )

    yl_slam_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("lvins_ros"),
                "launch",
                "odometry_node_launch.py",
            )
        ),
        launch_arguments={
            "odometry_log_path": f400_log_path,
            "odometry_imu_topic": LaunchConfiguration("f400_imu_topic"),
            "odometry_point_cloud_topics": LaunchConfiguration(
                "f400_point_cloud_topics"
            ),
        }.items(),
    )
    las_saver_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("lvins_ros"),
                "launch",
                "las_saver_node_launch.py",
            )
        ),
        launch_arguments={
            "las_save_path": f400_log_path,
        }.items(),
    )
    raw_data_recorder_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("lvins_ros"),
                "launch",
                "data_recorder_node_launch.py",
            )
        ),
        launch_arguments={
            "record_save_path": f400_log_path,
            "raw_imu_topic": LaunchConfiguration("f400_imu_topic"),
            "raw_point_cloud_topics": LaunchConfiguration("f400_point_cloud_topics"),
        }.items(),
    )

    return LaunchDescription(
        [
            f400_log_path_arg,
            f400_imu_topic_arg,
            f400_point_cloud_topics_arg,
            yl_slam_node,
            las_saver_node,
            raw_data_recorder_node,
        ]
    )
