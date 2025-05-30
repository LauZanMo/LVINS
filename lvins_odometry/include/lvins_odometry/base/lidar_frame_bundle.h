#pragma once

#include "lvins_common/nav_state.h"
#include "lvins_odometry/base/lidar_frame.h"

namespace lvins {

/**
 * @brief 雷达帧束类
 * @details 雷达帧束是指同一时刻所有雷达帧的集合，雷达帧束中所有雷达帧的时间戳相同
 */
class LidarFrameBundle : public NonCopyable {
public:
    using Ptr = std::shared_ptr<LidarFrameBundle>;

    /**
     * @brief 构造函数
     * @param frames 同一时刻的所有帧的集合
     */
    explicit LidarFrameBundle(const std::vector<LidarFrame::Ptr> &frames);

    /**
     * @brief 默认析构函数
     */
    ~LidarFrameBundle() = default;

    /**
     * @brief 获取帧束时间戳
     * @return 帧束时间戳
     */
    [[nodiscard]] int64_t timestamp() const;

    /**
     * @brief 获取帧束id
     * @return 帧束id
     */
    [[nodiscard]] long id() const;

    /**
     * @brief 获取帧束导航状态
     * @return 帧束导航状态
     */
    [[nodiscard]] const NavState &state() const;

    /**
     * @brief 设置帧束导航状态
     * @param state 帧束导航状态
     */
    void setState(const NavState &state);

    /**
     * @brief 获取帧束位姿
     * @return 帧束位姿
     */
    [[nodiscard]] const SE3f &Twb() const;

    /**
     * @brief 设置帧束位姿
     * @param T_wb 帧束位姿
     */
    void setTwb(const SE3f &T_wb);

    /**
     * @brief 获取帧束中各帧的外参
     * @return 帧束中各帧的外参
     */
    [[nodiscard]] std::vector<SE3f> Tbs() const;

    /**
     * @brief 设置帧束中各帧的外参
     * @param T_bs_vec 帧束中各帧的外参
     */
    void setTbs(const std::vector<SE3f> &T_bs_vec);

    /**
     * @brief 获取帧束中帧的数量
     * @return 帧束中帧的数量
     */
    [[nodiscard]] size_t size() const;

    /**
     * @brief 获取指定索引下的帧
     * @param idx 指定索引
     * @return 指定索引下的帧
     */
    [[nodiscard]] LidarFrame &frame(size_t idx);

    /**
     * @brief 获取指定索引下的帧
     * @param idx 指定索引
     * @return 指定索引下的帧
     */
    [[nodiscard]] const LidarFrame &frame(size_t idx) const;

    /**
     * @brief 获取帧束中的点云指针集合
     * @return 帧束中的点云指针集合
     */
    [[nodiscard]] std::vector<const PointCloud *> pointClouds() const;

    /**
     * @brief 打印帧束信息
     * @return 帧束信息
     */
    [[nodiscard]] std::string print() const;

private:
    long id_;                             ///< 帧束id（历史唯一）
    NavState state_;                      ///< 世界坐标系下的导航状态
    std::vector<LidarFrame::Ptr> frames_; ///< 帧束中所有帧的集合（帧的时间戳相同）
};

} // namespace lvins

/**
 * @brief 雷达帧束格式化器
 */
template<>
struct LVINS_FORMATTER<lvins::LidarFrameBundle> {
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
     * @param frame_bundle 雷达帧束
     * @param ctx 输出的格式化文本
     * @return 输出格式化文本的尾部迭代器
     */
    static auto format(const lvins::LidarFrameBundle &frame_bundle, LVINS_FORMAT_CONTEXT &ctx) {
        return LVINS_FORMAT_TO(ctx.out(), "{}", frame_bundle.print());
    }
};
