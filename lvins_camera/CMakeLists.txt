cmake_minimum_required(VERSION 3.15)
project(lvins_camera)

# 查找第三方库
find_package(OpenCV 4.2.0 REQUIRED)

# 查找ROS包
find_package(ament_cmake REQUIRED)
find_package(lvins_config REQUIRED)
find_package(lvins_common REQUIRED)

# 通用设置
lvins_setup()

# 添加库
add_library(${PROJECT_NAME} SHARED
    src/camera_geometry_base.cpp
    src/camera_rig.cpp
    src/distortion/no_distortion.cpp
    src/distortion/radial_tangential_distortion.cpp
    src/distortion/equidistant_distortion.cpp
    src/yaml/camera_yaml_serialization.cpp
    src/yaml/camera_rig_yaml_serialization.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>"
)
target_link_libraries(${PROJECT_NAME}
    ${OpenCV_LIBS}
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
    OpenCV
    lvins_config
    lvins_common
)
ament_package()