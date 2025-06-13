#!/usr/bin/env python3

import os

import laspy
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_srvs.srv import Trigger


class LasSaverNode(Node):
    """
    LAS文件保存节点
    该节点订阅点云消息并将其保存为LAS文件
    支持通过服务开始和停止存储点云数据
    订阅的点云消息主题可以通过参数配置

    支持的参数：
    - save_path: 保存LAS文件的路径
    - las_prefix: LAS文件的前缀
    - point_cloud_topics: 点云消息的主题列表

    使用方法：
    1. 启动节点
    2. 通过服务调用`~/start`开始存储点云数据
    3. 通过服务调用`~/stop`停止存储点云数据并保存LAS文件
    4. LAS文件将保存在指定的路径下，文件名格式为`<las_prefix><index>.las`
    其中`<index>`是存储的点云数据的索引，从0开始递增

    示例：
    ros2 run lvins_ros las_saver_node --ros-args -p save_path:=/path/to/save -p las_prefix:=map -p point_cloud_topics:=[/point_cloud1, /point_cloud2]
    该节点将订阅`/point_cloud1`和`/point_cloud2`主题的点云消息，并将其保存为LAS文件
    在`/path/to/save`目录下，文件名格式为`map0.las`, `map1.las`等

    注意事项：
    - 确保指定的保存路径存在或可以创建
    - 确保点云消息的格式正确，包含`x`, `y`, `z`, `intensity`字段
    - 如果存储过程中发生错误，将在日志中记录相关信息
    """

    def __init__(self):
        """
        构造函数
        """
        super().__init__("las_saver")
        self.get_logger().info("Starting LAS saver...")

        # 获取参数
        self.declare_parameter("save_path", "")
        self.declare_parameter("las_prefix", "map")
        self.declare_parameter("point_cloud_topics", ["/point_cloud"])
        self.save_path = (
            self.get_parameter("save_path").get_parameter_value().string_value
        )
        self.las_prefix = (
            self.get_parameter("las_prefix").get_parameter_value().string_value
        )
        self.point_cloud_topics = (
            self.get_parameter("point_cloud_topics")
            .get_parameter_value()
            .string_array_value
        )
        self.get_logger().info("LAS file save path: " + self.save_path)
        self.get_logger().info("LAS file prefix: " + self.las_prefix)
        self.get_logger().info("Point cloud topics: " + str(self.point_cloud_topics))

        # 初始化缓存和标志位
        self.storaging = False
        self.index = 0
        self.x_list = []
        self.y_list = []
        self.z_list = []
        self.intensity_list = []

        # 初始化订阅
        for topic in self.point_cloud_topics:
            self.create_subscription(
                msg_type=PointCloud2,
                topic=topic,
                callback=self.point_cloud_callback,
                qos_profile=1000,
            )

        # 初始化服务
        self.start_srv = self.create_service(
            Trigger, "~/start", self.start_storaging_callback
        )
        self.stop_srv = self.create_service(
            Trigger, "~/stop", self.stop_storaging_callback
        )

        self.get_logger().info("LAS saver started!")

    def point_cloud_callback(self, msg):
        """
        点云回调函数
        :param msg: 点云消息
        :return: None

        如果存储标志位为真则更新缓存，否则跳过
        """
        # 检测是否还未存储
        if not self.storaging:
            return

        # 更新缓存
        for point in point_cloud2.read_points_list(
            msg, field_names=("x", "y", "z", "intensity"), skip_nans=True
        ):
            # NED->ENU
            self.x_list.append(point[1])
            self.y_list.append(point[0])
            self.z_list.append(-point[2])
            self.intensity_list.append(point[3])

    def start_storaging_callback(self, _, response):
        """
        开始存储回调函数

        :param request: 请求（空）
        :param response: 响应
        :return: 响应

        如果存储标志位为真则直接拒绝，否则使能存储标志位，并返回响应
        """
        # 检测是否正在存储
        if self.storaging:
            self.get_logger().warn(
                "Fail to start storaging since it is already started."
            )
            response.success = False
            response.message = "Storaging is already started."
            return response

        # 设置标志位
        self.get_logger().info("Start storaging...")
        self.storaging = True

        # 返回响应
        response.success = True
        response.message = "Start storaging successfully."
        return response

    def stop_storaging_callback(self, _, response):
        """
        停止存储回调函数

        :param request: 请求（空）
        :param response: 响应
        :return: 响应

        如果存储标志位为假则直接拒绝，否则重置存储标志位，保存LAS文件并返回响应
        """
        # 检测是否还未存储
        if not self.storaging:
            self.get_logger().warn("Fail to stop storaging since it is not started.")
            response.success = False
            response.message = "Storaging is not started."
            return response

        # 设置标志位
        self.get_logger().info("Stop storaging...")
        self.storaging = False

        # 保存LAS文件
        self.save_las()

        # 返回响应
        response.success = True
        response.message = "Stop storaging successfully."
        return response

    def save_las(self):
        """
        保存LAS文件

        :return: None

        生成文件名、保存LAS文件、清空缓存并更新索引
        """
        # 生成文件名
        self.save_path = (
            self.get_parameter("save_path").get_parameter_value().string_value
        )
        self.las_prefix = (
            self.get_parameter("las_prefix").get_parameter_value().string_value
        )
        uri = os.path.join(self.save_path, f"{self.las_prefix}{self.index}.las")
        self.get_logger().info("Saving LAS file in: " + uri)

        # 保存LAS文件
        os.makedirs(self.save_path, exist_ok=True)
        las = laspy.create(file_version="1.2", point_format=3)
        las.x = np.array(self.x_list)
        las.y = np.array(self.y_list)
        las.z = np.array(self.z_list)
        las.intensity = np.array(self.intensity_list)
        las.write(uri)
        self.get_logger().info("Save LAS file successfully.")

        # 清空缓存
        self.x_list.clear()
        self.y_list.clear()
        self.z_list.clear()
        self.intensity_list.clear()

        # 更新索引
        self.index += 1


def main(args=None):
    """
    主函数
    :param args: 命令行参数
    :return: None
    """
    try:
        rclpy.init(args=args)
        las_saver = LasSaverNode()
        rclpy.spin(las_saver)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == "__main__":
    main()
