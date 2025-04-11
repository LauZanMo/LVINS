#pragma once

#include "lvins_common/eigen_types.h"
#include "lvins_common/non_copyable.h"
#include "lvins_lidar/lidar_geometry_base.h"

namespace lvins {

/**
 * @brief 雷达组类
 * @details 雷达组类用于管理多个雷达实例及其对应的外参
 */
class LidarRig : public NonCopyable {
public:
    using sPtr = std::shared_ptr<LidarRig>;

    /**
     * @brief 构造函数
     * @param label 雷达组标签
     * @param lidars 雷达组中的所有的雷达实例
     * @param T_bs_vec 雷达组中每个雷达的外参
     * @warning 输入的雷达实例和外参向量的长度需要一致
     */
    LidarRig(std::string label, const std::vector<LidarGeometryBase::sPtr> &lidars, std::vector<SE3f> T_bs_vec);

    /**
     * @brief 析构函数
     */
    ~LidarRig() = default;

    /**
     * @brief 从YAML配置文件中加载雷达组
     * @param config_file YAML配置文件路径（可以是相对路径）
     * @return 加载雷达组
     * @warning 如果加载失败，则返回空指针
     */
    static sPtr loadFromYaml(const std::string &config_file);

    /**
     * @brief 将雷达组参数写入YAML配置文件
     * @param config_file YAML配置文件路径（可以是相对路径）
     */
    void writeToYaml(const std::string &config_file) const;

    /**
     * @brief 获取雷达组标签
     * @return 雷达组标签
     */
    [[nodiscard]] const std::string &label() const;

    /**
     * @brief 获取雷达组中的指定索引的雷达实例
     * @param idx 雷达索引
     * @return 雷达实例
     */
    [[nodiscard]] const LidarGeometryBase::sPtr &lidar(size_t idx) const;

    /**
     * @brief 获取雷达组中实例的总数
     * @return 雷达实例总数
     */
    [[nodiscard]] size_t size() const;

    /**
     * @brief 获取雷达组中指定索引的雷达外参
     * @param idx 雷达索引
     * @return 雷达外参
     */
    [[nodiscard]] const SE3f &Tbs(size_t idx) const;

    /**
     * @brief 设置雷达组中指定索引的雷达外参
     * @param idx 雷达索引
     * @param T_bs 雷达外参
     */
    void setTbs(size_t idx, const SE3f &T_bs);

    /**
     * @brief 获取雷达组中所有雷达的外参
     * @return 雷达组中所有雷达的外参
     */
    [[nodiscard]] const std::vector<SE3f> &Tbs() const;

    /**
     * @brief 设置雷达组中所有雷达的外参
     * @param T_bs 雷达组中所有雷达的外参
     */
    void setTbs(const std::vector<SE3f> &T_bs);

    /**
     * @brief 打印雷达组参数
     * @return 雷达组参数
     */
    [[nodiscard]] std::string print() const;

private:
    std::string label_;
    std::vector<LidarGeometryBase::sPtr> lidars_;
    std::vector<SE3f> T_bs_vec_;
};

} // namespace lvins

/**
 * @brief 雷达组格式化器
 */
template<>
struct LVINS_FORMATTER<lvins::LidarRig> {
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
     * @param lidar_rig 雷达组
     * @param ctx 输出的格式化文本
     * @return 输出格式化文本的尾部迭代器
     */
    auto format(const lvins::LidarRig &lidar_rig, LVINS_FORMAT_CONTEXT &ctx) const {
        return LVINS_FORMAT_TO(ctx.out(), "{}", lidar_rig.print());
    }
};
