#pragma once

#include "lvins_common/eigen_types.h"

namespace lvins {

/**
 * @brief 全向相机的投影模型类（可用于鱼眼相机、全景相机等）
 * @details 该类为相机投影模型的全向相机实现
 * @tparam Distortion 相机的畸变模型
 */
template<typename Distortion>
class OmniProjection {
public:
    /**
     * @brief 构造函数
     * @param intrinsics 相机内参向量，顺序[xi, fx, fy, cx, cy]
     * @param distortion 相机的畸变模型
     */
    OmniProjection(const VecXf &intrinsics, const Distortion &distortion);

    /**
     * @brief 析构函数
     */
    ~OmniProjection() = default;

    /**
     * @brief 将三维点投影到像平面，并输出二维像素坐标
     * @param point 三维点
     * @param out_keypoint 二维像素坐标
     * @return 投影是否成功
     */
    bool project(const Eigen::Ref<const Vec3f> &point, Eigen::Ref<Vec2f> &out_keypoint) const;

    /**
     * @brief 将三维点投影到像平面，并输出二维像素坐标/三维点对二维像素坐标的雅可比矩阵
     * @param point 三维点
     * @param out_keypoint 二维像素坐标
     * @param out_jacobian 二维像素坐标对三维点的雅可比矩阵
     * @return 投影是否成功
     */
    bool project(const Eigen::Ref<const Vec3f> &point, Eigen::Ref<Vec2f> &out_keypoint, Mat23f &out_jacobian) const;

    /**
     * @brief 将二维像素坐标反投影到相机坐标系下，并输出三维点（仅方向，深度无实际意义）
     * @param keypoint 二维像素坐标
     * @param out_point 三维点
     * @return 反投影是否成功
     * @warning 返回的三维点不是单位向量
     */
    bool unproject(const Eigen::Ref<const Vec2f> &keypoint, Eigen::Ref<Vec3f> &out_point) const;

    /**
     * @brief 将二维像素坐标反投影到相机坐标系下，并输出三维点（仅方向，深度无实际意义）/二维像素坐标对三维点的雅可比矩阵
     * @param keypoint 二维像素坐标
     * @param out_point 三维点
     * @param out_jacobian 三维点对二维像素坐标的雅可比矩阵
     * @return 反投影是否成功
     * @warning 返回的三维点不是单位向量
     */
    bool unproject(const Eigen::Ref<const Vec2f> &keypoint, Eigen::Ref<Vec3f> &out_point, Mat32f &out_jacobian) const;

    /**
     * @brief 获取相机内参
     * @return 包含相机内参的向量
     */
    [[nodiscard]] VecXf intrinsicParameters() const;

    /**
     * @brief 获取相机畸变参数
     * @return 包含相机畸变参数的向量
     */
    [[nodiscard]] VecXf distortionParameters() const;

    /**
     * @brief 获取相机的畸变模型
     * @return 相机畸变模型
     */
    const Distortion &distortion() const;

    /**
     * @brief 打印投影模型参数
     * @return 投影模型参数
     */
    [[nodiscard]] std::string print() const;

private:
    [[nodiscard]] bool isRd2Valid(const Float &rd2) const;

    Float xi_;
    Float fx_;
    Float fy_;
    Float cx_;
    Float cy_;
    Float xi2_m1_inv_;
    Float fov_param_;
    Diag2f f_matrix_;
    Diag2f f_inv_matrix_;
    Vec2f c_vec_;
    Distortion distortion_;
};

} // namespace lvins

#include "lvins_camera/projection/omni_projection.hpp"
