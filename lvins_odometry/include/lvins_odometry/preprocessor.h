#pragma once

#include "lvins_common/non_copyable.h"
#include "lvins_common/string_helper.h"
#include "lvins_common/yaml/yaml_serialization.h"
#include "lvins_icp/point_cloud.h"

namespace lvins {

/**
 * @brief 预处理器类
 * @details 该类负责对原始点云和图像进行预处理
 */
class Preprocessor : public NonCopyable {
public:
    using Ptr = std::unique_ptr<Preprocessor>;

    /**
     * @brief 构造函数
     * @param config YAML配置节点
     */
    explicit Preprocessor(const YAML::Node &config);

    /**
     * @brief 将预处理器参数写入YAML节点
     * @return YAML节点
     */
    [[nodiscard]] YAML::Node writeToYaml() const;

    /**
     * @brief 处理原始点云
     * @param point_cloud 原始点云
     * @return 处理后的点云
     */
    [[nodiscard]] RawPointCloud::Ptr process(const RawPointCloud::ConstPtr &point_cloud) const;

    /**
     * @brief 打印预处理器参数接口
     * @return 预处理器参数
     */
    [[nodiscard]] std::string print() const;

private:
    float crop_box_size_;   ///< 裁剪尺寸（m）
    float voxel_grid_size_; ///< 体素尺寸（m）
};

} // namespace lvins

/**
 * @brief 预处理器格式化器
 */
template<>
struct LVINS_FORMATTER<lvins::Preprocessor> {
    /**
     * @brief 从文本中解析格式化字符
     * @param ctx 文本
     * @return 格式化字符尾部迭代器
     */
    static constexpr auto parse(const LVINS_FORMAT_PARSE_CONTEXT &ctx) {
        return ctx.begin();
    }

    /**
     * @brief 格式化
     * @param preprocessor 预处理器
     * @param ctx 输出的格式化文本
     * @return 输出格式化文本的尾部迭代器
     */
    static auto format(const lvins::Preprocessor &preprocessor, LVINS_FORMAT_CONTEXT &ctx) {
        return LVINS_FORMAT_TO(ctx.out(), "{}", preprocessor.print());
    }
};
