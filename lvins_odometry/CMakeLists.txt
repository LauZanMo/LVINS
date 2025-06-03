cmake_minimum_required(VERSION 3.15)
project(lvins_odometry)

# 查找ROS包
find_package(ament_cmake REQUIRED)
find_package(lvins_config REQUIRED)
find_package(lvins_common REQUIRED)
find_package(lvins_lidar REQUIRED)
find_package(lvins_camera REQUIRED)
find_package(lvins_icp REQUIRED)

# 通用设置
lvins_setup()

# 添加库
add_library(${PROJECT_NAME} SHARED
    # src/estimator.cpp
    src/drawer_base.cpp
    src/preprocessor.cpp
    src/base/lidar_frame.cpp
    src/base/lidar_frame_bundle.cpp
    src/init/initializer_base.cpp
    src/init/static_initializer.cpp
    # src/fusion/eskf.cpp
    # src/fusion/update_task.cpp
    # src/fusion/iterative_update_task.cpp
    # src/fusion/ins_helper.cpp
    # src/fusion/update_lidar_frame_bundle.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>"
)
ament_target_dependencies(${PROJECT_NAME}
    lvins_config
    lvins_common
    lvins_lidar
    lvins_camera
    lvins_icp
)

# 安装
install(TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib
    INCLUDES DESTINATION include
)

install(DIRECTORY include/
    DESTINATION include
)

ament_export_targets(${PROJECT_NAME})
ament_export_dependencies(
    lvins_config
    lvins_common
    lvins_lidar
    lvins_camera
    lvins_icp
)
ament_package()