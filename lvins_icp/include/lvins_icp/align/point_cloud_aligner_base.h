#pragma once

#include "lvins_common/noise_parameters.h"
#include "lvins_icp/align/result.h"
#include "lvins_icp/ann/nearest_neighbor_searcher.h"

namespace lvins {

/**
 * @brief 点云配准器类接口（抽象类）
 * @details 该类为点云配准器的接口类，所有点云配准器都通过YAML配置文件实现动态加载
 */
class PointCloudAlignerBase {
public:
    using Ptr    = std::shared_ptr<PointCloudAlignerBase>;
    using Result = point_cloud_align::Result;

    /**
     * @brief 默认构造函数
     */
    PointCloudAlignerBase() = default;

    /**
     * @brief 默认析构函数
     */
    virtual ~PointCloudAlignerBase() = default;

    /**
     * @brief 从YAML节点中加载点云配准器
     * @param config YAML节点
     * @return 加载点云配准器
     * @warning 如果加载失败，则返回空指针
     */
    static Ptr loadFromYaml(const YAML::Node &config);

    /**
     * @brief 将点云配准器参数写入YAML节点
     * @return YAML节点
     */
    [[nodiscard]] YAML::Node writeToYaml() const;

    /**
     * @brief 配准点云
     * @param target_nn_searcher 目标点云的最近邻搜索器
     * @param source_point_clouds 源点云集合
     * @param noise_params 噪声参数
     * @param init_T_tb 目标点云到载体的初始相对位姿
     * @param init_T_bs 初始雷达外参集合
     * @return 配准结果
     */
    virtual Result align(const NearestNeighborSearcher &target_nn_searcher,
                         const std::vector<const PointCloud *> &source_point_clouds,
                         const NoiseParameters &noise_params, const SE3f &init_T_tb,
                         const std::vector<SE3f> &init_T_bs) = 0;

    /**
     * @brief 线性化
     * @details 用于向外界（比如ESKF）提供指定状态下的线性化结果
     * @param target_nn_searcher 目标点云的最近邻搜索器
     * @param source_point_clouds 源点云集合
     * @param noise_params 噪声参数
     * @param T_tb 目标点云到载体的相对位姿
     * @param T_bs 雷达外参集合
     * @param H 信息矩阵
     * @param b 信息向量
     */
    virtual void linearize(const NearestNeighborSearcher &target_nn_searcher,
                           const std::vector<const PointCloud *> &source_point_clouds,
                           const NoiseParameters &noise_params, const SE3f &T_tb, const std::vector<SE3f> &T_bs,
                           MatXf &H, VecXf &b) = 0;

    /**
     * @brief 打印点云配准器参数
     * @return 点云配准器参数
     */
    [[nodiscard]] virtual std::string print() const = 0;
};

} // namespace lvins

/**
 * @brief 点云配准器格式化器
 * @tparam T 点云配准器派生类型
 * @tparam Char 格式化字符类型
 */
template<typename T, typename Char>
struct LVINS_FORMATTER<T, Char, std::enable_if_t<std::is_convertible_v<T *, lvins::PointCloudAlignerBase *>>> {
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
     * @param aligner 点云配准器
     * @param ctx 输出的格式化文本
     * @return 输出格式化文本的尾部迭代器
     */
    static auto format(const lvins::PointCloudAlignerBase &aligner, LVINS_FORMAT_CONTEXT &ctx) {
        return LVINS_FORMAT_TO(ctx.out(), "{}", aligner.print());
    }
};
