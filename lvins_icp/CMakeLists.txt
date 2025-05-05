cmake_minimum_required(VERSION 3.15)
project(lvins_icp)

# 查找第三方库
find_package(PCL REQUIRED)

# 查找ROS包
find_package(ament_cmake REQUIRED)
find_package(lvins_config REQUIRED)
find_package(lvins_common REQUIRED)

# 通用设置
lvins_setup()

# 添加库
add_library(${PROJECT_NAME} SHARED
    src/point_cloud.cpp
    src/ann/knn_result.cpp
    src/ann/flat_container.cpp
    src/ann/nearest_neighbor_searcher.cpp
    src/ann/incremental_voxel_map.cpp
    src/ann/yaml/nn_searcher_yaml_serialization.cpp
    src/preprocess/voxel_grid_sample.cpp
    src/preprocess/box_filter.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>"
)
target_link_libraries(${PROJECT_NAME}
    ${PCL_LIBRARIES}
)
target_compile_definitions(${PROJECT_NAME} PUBLIC
    PCL_NO_PRECOMPILE
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
    PCL
    lvins_config
    lvins_common
)
ament_package()