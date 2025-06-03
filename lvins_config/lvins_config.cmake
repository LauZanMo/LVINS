# 设置默认配置路径
set(LVINS_CONFIG_DIR "${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}")

# 设置记录等级，trace和debug会执行额外的检查，会影响性能
# 0:trace, 1:debug, 2:info, 3:warn, 4:error, 5:fatal
set(LVINS_LOG_LEVEL 0)

# 最大传感器数（仅相机或lidar）
set(LVINS_MAX_SENSOR_SIZE 16)

# 设置浮点数精度
set(LVINS_FLOAT_PRECISION float)

# 设置许可证检查
set(LVINS_LICENSE_CHECK true)
set(LVINS_PRELOAD_UTC 1749916800)
set(LVINS_PRELOAD_CPUID 1)

# 生成配置头文件
configure_file(
    ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/setup.h.in
    ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/setup.h
)