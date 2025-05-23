cmake_minimum_required(VERSION 3.15)
project(lvins_ros)

# 查找第三方库
list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")
find_package(LIBDW)
find_package(Backward)

# 查找ROS包
find_package(ament_cmake REQUIRED)
find_package(lvins_config REQUIRED)
find_package(lvins_common REQUIRED)
find_package(lvins_odometry REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(rosbag2_transport REQUIRED)

# 通用设置
lvins_setup()

# 添加可执行文件
add_executable(odometry_node
    src/odometry_node.cpp
    src/impl/drawer_rviz.cpp
    src/utils/point_cloud_converter.cpp
)
target_include_directories(odometry_node PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>"
)
target_link_libraries(odometry_node
    ${LIBDW_LIBRARIES}
    Backward::Backward
)
ament_target_dependencies(odometry_node
    lvins_config
    lvins_common
    lvins_odometry
    rclcpp
    std_srvs
    sensor_msgs
    geometry_msgs
    nav_msgs
    tf2_ros
    pcl_ros
    cv_bridge
    rosbag2_transport
)

# 安装
install(TARGETS odometry_node
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
    DESTINATION include
)

install(DIRECTORY
    launch
    DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(
    LIBDW
    Backward
    lvins_config
    lvins_common
    lvins_odometry
    rclcpp
    std_srvs
    sensor_msgs
    geometry_msgs
    nav_msgs
    tf2_ros
    pcl_ros
    cv_bridge
    rosbag2_transport
)
ament_package()