#pragma once

#include "lvins_common/non_copyable.h"
#include "lvins_icp/point_cloud.h"
#include "lvins_lidar/lidar_geometry_base.h"

namespace lvins {

/**
 * @brief 雷达帧类
 * @details 雷达帧类包含了帧的基本信息，如时间戳、传感器及相关数据等
 */
class LidarFrame : public NonCopyable {
public:
    using Ptr      = std::shared_ptr<LidarFrame>;
    using ConstPtr = std::shared_ptr<const LidarFrame>;

    /**
     * @brief 构造函数
     * @param timestamp 帧时间戳（ns）
     * @param lidar 帧所属的雷达
     * @param raw_point_cloud 帧原始点云
     */
    LidarFrame(int64_t timestamp, const LidarGeometryBase &lidar, RawPointCloud::Ptr raw_point_cloud);

    /**
     * @brief 默认析构函数
     */
    ~LidarFrame() = default;

    /**
     * @brief 获取帧时间戳（ns）
     * @return 帧时间戳
     */
    [[nodiscard]] int64_t timestamp() const;

    /**
     * @brief 获取帧id
     * @return 帧id
     */
    [[nodiscard]] long id() const;

    /**
     * @brief 获取帧所属的帧束id
     * @return 帧束id
     * @warning 如果返回-1，表示帧还没有被加入到帧束中
     */
    [[nodiscard]] long bundleId() const;

    /**
     * @brief 设置帧所属的帧束id
     * @param bundle_id 帧束id
     */
    void setBundleId(long bundle_id);

    /**
     * @brief 获取帧所属的雷达
     * @return 帧所属的雷达
     */
    [[nodiscard]] const LidarGeometryBase &lidar() const;

    /**
     * @brief 获取帧位姿
     * @return 帧位姿
     * @warning 帧创建时该值属于未设置状态，需要设置后才能使用
     */
    [[nodiscard]] const SE3f &Twf() const;

    /**
     * @brief 设置帧位姿
     * @param T_wf 帧位姿
     */
    void setTwf(const SE3f &T_wf);

    /**
     * @brief 获取帧所属传感器外参
     * @return 帧所属传感器外参
     * @warning 帧创建时该值属于未设置状态，需要设置后才能使用
     */
    [[nodiscard]] const SE3f &Tbs() const;

    /**
     * @brief 设置帧所属传感器外参
     * @param T_bs 帧所属传感器外参
     */
    void setTbs(const SE3f &T_bs);

    /**
     * @brief 获取帧原始点云
     * @return 帧原始点云
     */
    [[nodiscard]] RawPointCloud::Ptr &rawPointCloud();

    /**
     * @brief 获取帧原始点云
     * @return 帧原始点云
     */
    [[nodiscard]] RawPointCloud::ConstPtr rawPointCloud() const;

    /**
     * @brief 获取帧点云
     * @return 帧点云
     */
    [[nodiscard]] PointCloud::Ptr &pointCloud();

    /**
     * @brief 获取帧点云
     * @return 帧点云
     */
    [[nodiscard]] PointCloud::ConstPtr pointCloud() const;

    /**
     * @brief 打印帧信息
     * @return 帧信息
     */
    [[nodiscard]] std::string print() const;

private:
    // 帧信息
    int64_t timestamp_;                  ///< 时间戳（ns）
    long id_;                            ///< 帧id（历史唯一）
    long bundle_id_{-1};                 ///< 帧束id（用于多目，历史唯一）
    const LidarGeometryBase &lidar_;     ///< 帧所属的雷达
    SE3f T_wf_;                          ///< 世界坐标系到帧坐标系的变换
    SE3f T_bs_;                          ///< 帧所属传感器外参
    RawPointCloud::Ptr raw_point_cloud_; ///< 原始点云
    PointCloud::Ptr point_cloud_;        ///< 帧点云（去畸变后的点云）
};

} // namespace lvins

/**
 * @brief 雷达帧格式化器
 */
template<>
struct LVINS_FORMATTER<lvins::LidarFrame> {
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
     * @param frame 雷达帧
     * @param ctx 输出的格式化文本
     * @return 输出格式化文本的尾部迭代器
     */
    static auto format(const lvins::LidarFrame &frame, LVINS_FORMAT_CONTEXT &ctx) {
        return LVINS_FORMAT_TO(ctx.out(), "{}", frame.print());
    }
};
