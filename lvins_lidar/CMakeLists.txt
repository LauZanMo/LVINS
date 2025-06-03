cmake_minimum_required(VERSION 3.15)
project(lvins_lidar)

# 查找ROS包
find_package(ament_cmake REQUIRED)
find_package(lvins_config REQUIRED)
find_package(lvins_common REQUIRED)

# 通用设置
lvins_setup()

# 添加库
add_library(${PROJECT_NAME} SHARED
    src/lidar_geometry_base.cpp
    src/lidar_rig.cpp
    src/yaml/lidar_yaml_serialization.cpp
    src/yaml/lidar_rig_yaml_serialization.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>"
)
ament_target_dependencies(${PROJECT_NAME}
    lvins_config
    lvins_common
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
)
ament_package()