#pragma once

#include "lvins_camera/camera_geometry_base.h"

namespace lvins {

/**
 * @brief 相机几何类的模板类
 * @details 该类继承自相机几何抽象类，提供了相机几何类的模板实现
 * @tparam Projection 相机几何类的投影模型
 */
template<typename Projection>
class CameraGeometry final : public CameraGeometryBase {
public:
    /**
     * @brief 构造函数
     * @param label 相机标签
     * @param width 相机宽度（水平像素数）
     * @param height 相机高度（水平像素数）
     * @param mask 相机掩模
     * @param mask_file_name 相机掩模文件名
     * @param projection 相机的投影模型
     */
    CameraGeometry(std::string label, int width, int height, cv::Mat mask, std::string mask_file_name,
                   const Projection &projection);

    /**
     * @brief 析构函数
     */
    ~CameraGeometry() override = default;

    /**
     * @brief 将三维点投影到像平面，并输出二维像素坐标
     * @param point 三维点
     * @param out_keypoint 二维像素坐标
     * @return 投影是否成功
     */
    bool project(const Eigen::Ref<const Vec3f> &point, Eigen::Ref<Vec2f> &out_keypoint) const override;

    /**
     * @brief 将三维点投影到像平面，并输出二维像素坐标/三维点对二维像素坐标的雅可比矩阵
     * @param point 三维点
     * @param out_keypoint 二维像素坐标
     * @param out_jacobian 二维像素坐标对三维点的雅可比矩阵
     * @return 投影是否成功
     */
    bool project(const Eigen::Ref<const Vec3f> &point, Eigen::Ref<Vec2f> &out_keypoint,
                 Mat23f &out_jacobian) const override;

    /**
     * @brief 将二维像素坐标反投影到相机坐标系下，并输出三维点（仅方向，深度无实际意义）
     * @param keypoint 二维像素坐标
     * @param out_point 三维点
     * @return 反投影是否成功
     * @warning 返回的三维点不是单位向量
     */
    bool unproject(const Eigen::Ref<const Vec2f> &keypoint, Eigen::Ref<Vec3f> &out_point) const override;

    /**
     * @brief 将二维像素坐标反投影到相机坐标系下，并输出三维点（仅方向，深度无实际意义）/二维像素坐标对三维点的雅可比矩阵
     * @param keypoint 二维像素坐标
     * @param out_point 三维点
     * @param out_jacobian 三维点对二维像素坐标的雅可比矩阵
     * @return 反投影是否成功
     * @warning 返回的三维点不是单位向量
     */
    bool unproject(const Eigen::Ref<const Vec2f> &keypoint, Eigen::Ref<Vec3f> &out_point,
                   Mat32f &out_jacobian) const override;

    /**
     * @brief 获取相机id
     * @return 相机id
     */
    [[nodiscard]] int id() const override;

    /**
     * @brief 设置相机id
     * @param id 相机id
     */
    void setId(int id) override;

    /**
     * @brief 获取相机标签
     * @return 相机标签
     */
    [[nodiscard]] const std::string &label() const override;

    /**
     * @brief 获取相机宽度（水平像素数）
     * @return 相机宽度
     */
    [[nodiscard]] int width() const override;

    /**
     * @brief 获取相机高度（水平像素数）
     * @return 相机高度
     */
    [[nodiscard]] int height() const override;

    /**
     * @brief 获取相机是否有掩模
     * @return 相机是否有掩模
     */
    [[nodiscard]] bool hasMask() const override;

    /**
     * @brief 获取相机掩模
     * @return 相机掩模
     */
    [[nodiscard]] const cv::Mat &mask() const override;

    /**
     * @brief 获取二维像素坐标是否被掩模遮盖
     * @param keypoint 二维像素坐标
     * @return 二维像素坐标是否被掩模遮盖
     */
    [[nodiscard]] bool isMasked(const Eigen::Ref<const Vec2f> &keypoint) const override;

    /**
     * @brief 获取相机掩模文件名
     * @return 相机掩模文件名
     */
    [[nodiscard]] const std::string &maskFileName() const override;

    /**
     * @brief 获取相机内参
     * @return 包含相机内参的向量
     */
    [[nodiscard]] VecXf intrinsicParameters() const override;

    /**
     * @brief 获取相机畸变参数
     * @return 包含相机畸变参数的向量
     */
    [[nodiscard]] VecXf distortionParameters() const override;

    /**
     * @brief 获取相机投影模型
     * @return 相机投影模型
     */
    const Projection &projection() const;

    /**
     * @brief 打印相机参数
     * @return 相机参数
     */
    [[nodiscard]] std::string print() const override;

private:
    int id_{-1};
    std::string label_;
    int width_;
    int height_;
    cv::Mat mask_;
    std::string mask_file_name_;
    Projection projection_;
};

} // namespace lvins

#include "lvins_camera/camera_geometry.hpp"
