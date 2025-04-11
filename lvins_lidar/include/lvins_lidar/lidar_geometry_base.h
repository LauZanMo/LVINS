#pragma once

#include "lvins_common/eigen_types.h"
#include "lvins_common/string_helper.h"

#include <memory>

namespace lvins {

/**
 * @brief 雷达几何类
 * @details 该类为雷达的基础类，所有雷达都通过YAML配置文件实现动态加载
 */
class LidarGeometryBase {
public:
    using sPtr      = std::shared_ptr<LidarGeometryBase>;
    using sConstPtr = std::shared_ptr<const LidarGeometryBase>;

    /**
     * @brief 构造函数
     * @param label 雷达标签
     * @param scan_line 雷达线数
     * @param nearest_distance 雷达所测最近距离
     * @param farthest_distance 雷达所测最远距离
     */
    LidarGeometryBase(std::string label, uint32_t scan_line, Float nearest_distance, Float farthest_distance);

    /**
     * @brief 从YAML配置文件中加载雷达
     * @param config_file YAML配置文件路径（可以是相对路径）
     * @return 加载雷达的指针
     * @warning 如果加载失败，则返回空指针
     */
    static sPtr loadFromYaml(const std::string &config_file);

    /**
     * @brief 将雷达参数写入YAML配置文件
     * @param config_file YAML配置文件路径（可以是相对路径）
     */
    void writeToYaml(const std::string &config_file) const;

    /**
     * @brief 默认析构函数
     */
    ~LidarGeometryBase() = default;

    /**
     * @brief 获取雷达id
     * @return 雷达id
     */
    [[nodiscard]] int id() const;

    /**
     * @brief 设置雷达id
     * @param id 雷达id
     */
    void setId(int id);

    /**
     * @brief 获取雷达标签
     * @return 雷达标签
     */
    [[nodiscard]] const std::string &label() const;

    /**
     * @brief 获取雷达线数
     * @return 雷达线数
     */
    [[nodiscard]] uint32_t scanLine() const;

    /**
     * @brief 获取雷达测量最近距离
     * @return 雷达测量最近距离
     */
    [[nodiscard]] Float nearestDistance() const;

    /**
     * @brief 获取雷达测量最远距离
     * @return 雷达测量最远距离
     */
    [[nodiscard]] Float farthestDistance() const;

    /**
     * @brief 检查雷达所测三维点是否有效
     * @param point 雷达所测三维点
     * @return 雷达所测三维点是否有效
     */
    template<typename DerivedPoint>
    [[nodiscard]] bool isPointValid(const Eigen::MatrixBase<DerivedPoint> &point) const;

    /**
     * @brief 打印雷达参数
     * @return 雷达参数
     */
    [[nodiscard]] std::string print() const;

private:
    int id_{-1};
    std::string label_;
    uint32_t scan_line_;
    Float nearest_dist_;
    Float nearest_dist2_;
    Float farthest_dist_;
    Float farthest_dist2_;
};

} // namespace lvins

#include "lvins_lidar/lidar_geometry_base.hpp"

/**
 * @brief 雷达格式化器
 * @tparam T 雷达派生类型
 * @tparam Char 格式化字符类型
 */
template<typename T, typename Char>
struct LVINS_FORMATTER<T, Char, std::enable_if_t<std::is_convertible_v<T *, lvins::LidarGeometryBase *>>> {
    /**
     * @brief 从文本中解析格式化字符
     * @param ctx 文本
     * @return 格式化字符尾部迭代器
     */
    constexpr auto parse(LVINS_FORMAT_PARSE_CONTEXT &ctx) {
        return ctx.begin();
    }

    /**
     * @brief 格式化
     * @param lidar 雷达
     * @param ctx 输出的格式化文本
     * @return 输出格式化文本的尾部迭代器
     */
    auto format(const lvins::LidarGeometryBase &lidar, LVINS_FORMAT_CONTEXT &ctx) const {
        return LVINS_FORMAT_TO(ctx.out(), "{}", lidar.print());
    }
};
