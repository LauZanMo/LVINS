cmake_minimum_required(VERSION 3.15)
project(lvins_test)

# 查找ROS包
find_package(ament_cmake REQUIRED)
find_package(lvins_config REQUIRED)
find_package(lvins_common REQUIRED)
find_package(lvins_icp REQUIRED)

# 通用设置
lvins_setup()

# 添加库
add_executable(${PROJECT_NAME}
    src/test.cpp
    src/bfnn.cc
)
target_include_directories(${PROJECT_NAME} PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>"
)
ament_target_dependencies(${PROJECT_NAME}
    lvins_config
    lvins_common
    lvins_icp
)

# 安装
install(TARGETS ${PROJECT_NAME}
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
    DESTINATION include
)

ament_export_dependencies(
    lvins_config
    lvins_common
    lvins_icp
)
ament_package()