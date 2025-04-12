#pragma once

#include "lvins_camera/camera_geometry_base.h"
#include "lvins_common/eigen_types.h"
#include "lvins_common/non_copyable.h"

namespace lvins {

/**
 * @brief 相机组类
 * @details 相机组类用于管理多个相机实例及其对应的外参
 */
class CameraRig : NonCopyable {
public:
    using sPtr = std::shared_ptr<CameraRig>;

    /**
     * @brief 构造函数
     * @param label 相机组标签
     * @param cameras 相机组中的所有的相机实例
     * @param T_bs_vec 相机组中每个相机的外参
     * @warning 输入的相机实例和外参向量的长度需要一致
     */
    CameraRig(std::string label, const std::vector<CameraGeometryBase::sPtr> &cameras, std::vector<SE3f> T_bs_vec);

    /**
     * @brief 析构函数
     */
    ~CameraRig() = default;

    /**
     * @brief 从YAML配置文件中加载相机组
     * @param config_file YAML配置文件路径（可以是相对路径）
     * @return 加载相机组
     * @warning 如果加载失败，则返回空指针
     */
    static sPtr loadFromYaml(const std::string &config_file);

    /**
     * @brief 将相机组参数写入YAML配置文件
     * @param config_file YAML配置文件路径（可以是相对路径）
     */
    void writeToYaml(const std::string &config_file) const;

    /**
     * @brief 获取相机组标签
     * @return 相机组标签
     */
    [[nodiscard]] const std::string &label() const;

    /**
     * @brief 获取相机组中的指定索引的相机实例
     * @param idx 相机索引
     * @return 相机实例
     */
    [[nodiscard]] const CameraGeometryBase::sPtr &camera(size_t idx) const;

    /**
     * @brief 获取相机组中实例的总数
     * @return 相机实例总数
     */
    [[nodiscard]] size_t size() const;

    /**
     * @brief 获取相机组中指定索引的相机外参
     * @param idx 相机索引
     * @return 相机外参
     */
    [[nodiscard]] const SE3f &Tbs(size_t idx) const;

    /**
     * @brief 设置相机组中指定索引的相机外参
     * @param idx 相机索引
     * @param T_bs 相机外参
     */
    void setTbs(size_t idx, const SE3f &T_bs);

    /**
     * @brief 获取相机组中所有相机的外参
     * @return 相机组中所有相机的外参
     */
    [[nodiscard]] const std::vector<SE3f> &Tbs() const;

    /**
     * @brief 设置相机组中所有相机的外参
     * @param T_bs 相机组中所有相机的外参
     */
    void setTbs(const std::vector<SE3f> &T_bs);

    /**
     * @brief 打印相机组参数
     * @return 相机组参数
     */
    [[nodiscard]] std::string print() const;

private:
    std::string label_;
    std::vector<CameraGeometryBase::sPtr> cameras_;
    std::vector<SE3f> T_bs_vec_;
};

} // namespace lvins

/**
 * @brief 相机组格式化器
 */
template<>
struct LVINS_FORMATTER<lvins::CameraRig> {
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
     * @param camera_rig 相机组
     * @param ctx 输出的格式化文本
     * @return 输出格式化文本的尾部迭代器
     */
    auto format(const lvins::CameraRig &camera_rig, LVINS_FORMAT_CONTEXT &ctx) const {
        return LVINS_FORMAT_TO(ctx.out(), "{}", camera_rig.print());
    }
};
