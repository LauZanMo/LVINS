#pragma once

#include "lvins_common/eigen_types.h"
#include "lvins_common/logger.h"

#include <vector>

namespace lvins::point_cloud_align {

/**
 * @brief 点云配准结果类
 */
struct Result {
    /**
     * @brief 默认构造函数
     */
    Result() = default;

    /**
     * @brief 默认析构函数
     */
    ~Result() = default;

    /**
     * @brief 打印点云配准结果
     * @return 点云配准结果
     */
    [[nodiscard]] std::string print() const;

    SE3f T_tb;              ///< 目标点云到载体的相对位姿
    std::vector<SE3f> T_bs; ///< 雷达外参集合

    size_t iterations{0}; ///< 迭代次数
};

} // namespace lvins::point_cloud_align

/**
 * @brief 点云配准结果格式化器
 */
template<>
struct LVINS_FORMATTER<lvins::point_cloud_align::Result> {
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
     * @param result 点云配准结果
     * @param ctx 输出的格式化文本
     * @return 输出格式化文本的尾部迭代器
     */
    static auto format(const lvins::point_cloud_align::Result &result, LVINS_FORMAT_CONTEXT &ctx) {
        return LVINS_FORMAT_TO(ctx.out(), "{}", result.print());
    }
};
