cmake_minimum_required(VERSION 3.15)
project(lvins_common)

# 查找第三方库
find_package(spdlog 1.5.0 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Sophus REQUIRED)

# 查找ROS包
find_package(ament_cmake REQUIRED)
find_package(lvins_config REQUIRED)

# 通用设置
lvins_setup()

# 添加库
add_library(${PROJECT_NAME} SHARED
    src/eigen_helper.cpp
    src/logger.cpp
    src/nav_state.cpp
    src/noise_parameters.cpp
    src/path_helper.cpp
    src/rotation_helper.cpp
    src/time/timer.cpp
    src/time/time_task.cpp
    src/time/time_wheel.cpp
    src/time/time_wheel_scheduler.cpp
    src/sensor/imu.cpp
    src/yaml/yaml_serialization.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>"
)
target_link_libraries(${PROJECT_NAME}
    spdlog::spdlog
    ${YAML_CPP_LIBRARIES}
    Sophus::Sophus
)
ament_target_dependencies(${PROJECT_NAME}
    lvins_config
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
    spdlog
    yaml-cpp
    Sophus
    lvins_config
)
ament_package()