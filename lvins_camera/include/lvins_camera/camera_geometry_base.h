#pragma once

#include "lvins_common/eigen_types.h"
#include "lvins_common/string_helper.h"

#include <memory>
#include <opencv2/core.hpp>

namespace lvins {

/**
 * @brief 相机几何类接口（抽象类）
 * @details 该类为相机的接口类，所有相机都通过YAML配置文件实现动态加载
 */
class CameraGeometryBase {
public:
    using sPtr      = std::shared_ptr<CameraGeometryBase>;
    using sConstPtr = std::shared_ptr<const CameraGeometryBase>;

    /**
     * @brief 默认构造函数与析构函数
     */
    CameraGeometryBase()          = default;
    virtual ~CameraGeometryBase() = default;

    /**
     * @brief 从YAML配置文件中加载相机
     * @param config_file YAML配置文件路径（可以是相对路径）
     * @return 加载相机
     * @warning 如果加载失败，则返回空指针
     */
    static sPtr loadFromYaml(const std::string &config_file);

    /**
     * @brief 将相机参数写入YAML配置文件
     * @param config_file YAML配置文件路径（可以是相对路径）
     */
    void writeToYaml(const std::string &config_file) const;

    /**
     * @brief 将三维点投影到像平面，并输出二维像素坐标
     * @param point 三维点
     * @param out_keypoint 二维像素坐标
     * @return 投影是否成功
     */
    virtual bool project(const Eigen::Ref<const Vec3f> &point, Eigen::Ref<Vec2f> &out_keypoint) const = 0;

    /**
     * @brief 将三维点投影到像平面，并输出二维像素坐标/三维点对二维像素坐标的雅可比矩阵
     * @param point 三维点
     * @param out_keypoint 二维像素坐标
     * @param out_jacobian 二维像素坐标对三维点的雅可比矩阵
     * @return 投影是否成功
     */
    virtual bool project(const Eigen::Ref<const Vec3f> &point, Eigen::Ref<Vec2f> &out_keypoint,
                         Mat23f &out_jacobian) const = 0;

    /**
     * @brief 将二维像素坐标反投影到相机坐标系下，并输出三维点（仅方向，深度无实际意义）
     * @param keypoint 二维像素坐标
     * @param out_point 三维点
     * @return 反投影是否成功
     * @warning 返回的三维点不是单位向量
     */
    virtual bool unproject(const Eigen::Ref<const Vec2f> &keypoint, Eigen::Ref<Vec3f> &out_point) const = 0;

    /**
     * @brief 将二维像素坐标反投影到相机坐标系下，并输出三维点（仅方向，深度无实际意义）/二维像素坐标对三维点的雅可比矩阵
     * @param keypoint 二维像素坐标
     * @param out_point 三维点
     * @param out_jacobian 三维点对二维像素坐标的雅可比矩阵
     * @return 反投影是否成功
     * @warning 返回的三维点不是单位向量
     */
    virtual bool unproject(const Eigen::Ref<const Vec2f> &keypoint, Eigen::Ref<Vec3f> &out_point,
                           Mat32f &out_jacobian) const = 0;

    /**
     * @brief 获取相机id
     * @return 相机id
     */
    [[nodiscard]] virtual int id() const = 0;

    /**
     * @brief 设置相机id
     * @param id 相机id
     */
    virtual void setId(int id) = 0;

    /**
     * @brief 获取相机标签
     * @return 相机标签
     */
    [[nodiscard]] virtual const std::string &label() const = 0;

    /**
     * @brief 获取相机宽度（水平像素数）
     * @return 相机宽度
     */
    [[nodiscard]] virtual int width() const = 0;

    /**
     * @brief 获取相机高度（水平像素数）
     * @return 相机高度
     */
    [[nodiscard]] virtual int height() const = 0;

    /**
     * @brief 获取相机是否有掩模
     * @return 相机是否有掩模
     */
    [[nodiscard]] virtual bool hasMask() const = 0;

    /**
     * @brief 获取相机掩模
     * @return 相机掩模
     */
    [[nodiscard]] virtual const cv::Mat &mask() const = 0;

    /**
     * @brief 获取二维像素坐标是否被掩模遮盖
     * @param keypoint 二维像素坐标
     * @return 二维像素坐标是否被掩模遮盖
     */
    [[nodiscard]] virtual bool isMasked(const Eigen::Ref<const Vec2f> &keypoint) const = 0;

    /**
     * @brief 获取掩模文件名
     * @return 掩模文件名
     */
    [[nodiscard]] virtual const std::string &maskFileName() const = 0;

    /**
     * @brief 检查二维像素坐标是否在像平面可见
     * @tparam DerivedKeypoint 二维像素坐标的类型
     * @param keypoint 二维像素坐标
     * @return 二维像素坐标是否在像平面可见
     */
    template<typename DerivedKeypoint>
    bool isKeypointVisible(const Eigen::MatrixBase<DerivedKeypoint> &keypoint) const;

    /**
     * @brief 检查二维像素坐标是否在像平面可见（去掉边缘）
     * @tparam DerivedKeypoint 二维像素坐标的类型
     * @param keypoint 二维像素坐标
     * @param border 边缘厚度
     * @return 二维像素坐标是否在像平面可见（去掉边缘）
     */
    template<typename DerivedKeypoint>
    bool isKeypointVisible(const Eigen::MatrixBase<DerivedKeypoint> &keypoint,
                           typename DerivedKeypoint::Scalar border) const;

    /**
     * @brief 获取相机内参
     * @return 包含相机内参的向量
     */
    [[nodiscard]] virtual VecXf intrinsicParameters() const = 0;

    /**
     * @brief 获取相机畸变参数
     * @return 包含相机畸变参数的向量
     */
    [[nodiscard]] virtual VecXf distortionParameters() const = 0;

    /**
     * @brief 打印相机参数
     * @return 相机参数
     */
    [[nodiscard]] virtual std::string print() const = 0;
};

} // namespace lvins

#include "lvins_camera/camera_geometry_base.hpp"

/**
 * @brief 相机格式化器
 * @tparam T 相机派生类型
 * @tparam Char 格式化字符类型
 */
template<typename T, typename Char>
struct LVINS_FORMATTER<T, Char, std::enable_if_t<std::is_convertible_v<T *, lvins::CameraGeometryBase *>>> {
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
     * @param camera 相机
     * @param ctx 输出的格式化文本
     * @return 输出格式化文本的尾部迭代器
     */
    static auto format(const lvins::CameraGeometryBase &camera, LVINS_FORMAT_CONTEXT &ctx) {
        return LVINS_FORMAT_TO(ctx.out(), "{}", camera.print());
    }
};
