#pragma once

#include "lvins_common/sensor/imu.h"
#include "lvins_common/yaml/yaml_serialization.h"
#include "lvins_icp/align/point_cloud_aligner.h"
#include "lvins_odometry/base/lidar_frame_bundle.h"

namespace lvins {

/**
 * @brief 初始化器基类
 * @details 该类为初始化器的基础类，所有初始化器都通过YAML配置文件实现动态加载
 */
class InitializerBase : public NonCopyable {
public:
    using Ptr = std::unique_ptr<InitializerBase>;

    /**
     * @brief 构造函数
     * @param noise_params 噪声参数
     * @param aligner 点云配准器
     * @param g_w 世界坐标系下的重力向量
     */
    InitializerBase(const NoiseParameters &noise_params, const PointCloudAligner &aligner, Vec3f g_w);

    /**
     * @brief 默认析构函数
     */
    virtual ~InitializerBase() = default;

    /**
     * @brief 从YAML节点中加载初始化器
     * @param config YAML节点
     * @param noise_params 噪声参数
     * @param aligner 点云配准器
     * @param g_w 世界坐标系下的重力向量
     * @return 所加载的初始化器
     */
    static Ptr loadFromYaml(const YAML::Node &config, const NoiseParameters &noise_params,
                            const PointCloudAligner &aligner, Vec3f g_w);

    /**
     * @brief 将初始化器参数写入YAML节点
     * @return YAML节点
     */
    [[nodiscard]] YAML::Node writeToYaml() const;

    /**
     * @brief 尝试初始化
     * @param lidar_frame_bundle 新的雷达帧束
     * @param imus 新的IMU数据
     * @return 是否初始化成功
     */
    [[nodiscard]] virtual bool tryInitialize(const LidarFrameBundle::Ptr &lidar_frame_bundle, const Imus &imus) = 0;

    /**
     * @brief 获取雷达帧束容器
     * @return 雷达帧束容器
     * @warning 该方法在初始化完成后才可调用
     */
    [[nodiscard]] virtual const std::vector<LidarFrameBundle::Ptr> &lidarFrameBundles() const = 0;

    /**
     * @brief 获取导航状态
     * @return 导航状态
     * @warning 该方法在初始化完成后才可调用
     */
    [[nodiscard]] virtual const NavState &navState() const = 0;

    /**
     * @brief 重置初始化器
     */
    virtual void reset() = 0;

    /**
     * @brief 获取初始化器类型
     * @return 初始化器类型
     */
    [[nodiscard]] virtual std::string type() const = 0;

    /**
     * @brief 获取初始化器参数
     * @return 初始化器参数
     */
    [[nodiscard]] virtual VecXf parameters() const = 0;

    /**
     * @brief 打印初始化器参数
     * @return 初始化器参数
     */
    [[nodiscard]] virtual std::string print() const = 0;

protected:
    const NoiseParameters &noise_params_; ///< 噪声参数
    const PointCloudAligner &aligner_;    ///< 点云配准器
    Vec3f g_w_;                           ///< 世界坐标系下的重力向量
    bool initialized_{false};             ///< 初始化标志位
};

} // namespace lvins

/**
 * @brief 初始化器格式化器
 * @tparam T 初始化器派生类型
 * @tparam Char 格式化字符类型
 */
template<typename T, typename Char>
struct LVINS_FORMATTER<T, Char, std::enable_if_t<std::is_convertible_v<T *, lvins::InitializerBase *>>> {
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
     * @param initializer 初始化器
     * @param ctx 输出的格式化文本
     * @return 输出格式化文本的尾部迭代器
     */
    static auto format(const lvins::InitializerBase &initializer, LVINS_FORMAT_CONTEXT &ctx) {
        return LVINS_FORMAT_TO(ctx.out(), "{}", initializer.print());
    }
};
