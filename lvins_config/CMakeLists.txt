cmake_minimum_required(VERSION 3.15)
project(lvins_config)

# 查找ROS包
find_package(ament_cmake REQUIRED)

# 生成设置头文件
include(lvins_config.cmake)

# 添加库
add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME} INTERFACE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>"
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
    FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY
    cmake
    config
    DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(${PROJECT_NAME})
ament_package(
    CONFIG_EXTRAS "lvins_config-extras.cmake"
)
