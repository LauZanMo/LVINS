#pragma once

#include "lvins_common/eigen_types.h"

namespace lvins {

/**
 * @brief 径向-切向畸变的畸变模型类
 * @details 该类为畸变模型的径向-切向畸变实现
 */
class RadialTangentialDistortion {
public:
    /**
     * @brief 构造函数
     * @param parameters 相机畸变参数向量，顺序[k1, k2, p1, p2]
     */
    explicit RadialTangentialDistortion(const VecXf &parameters);

    /**
     * @brief 析构函数
     */
    ~RadialTangentialDistortion() = default;

    /**
     * @brief 将无畸变的归一化平面坐标转换为畸变的归一化平面坐标
     * @param uv 无畸变的归一化平面坐标
     * @return 畸变的归一化平面坐标
     */
    [[nodiscard]] Vec2f distort(const Vec2f &uv) const;

    /**
     * @brief 将畸变的归一化平面坐标转换为无畸变的归一化平面坐标
     * @param uv 畸变的归一化平面坐标
     * @return 无畸变的归一化平面坐标
     */
    [[nodiscard]] Vec2f undistort(const Vec2f &uv) const;

    /**
     * @brief 计算畸变的归一化平面坐标对无畸变的归一化平面坐标的雅可比矩阵
     * @return 畸变的归一化平面坐标对无畸变的归一化平面坐标的雅可比矩阵
     */
    [[nodiscard]] Mat22f jacobian(const Vec2f &uv) const;

    /**
     * @brief 获取相机畸变参数
     * @return 包含相机畸变参数的向量
     */
    [[nodiscard]] VecXf distortionParameters() const;

    /**
     * @brief 打印畸变模型参数
     * @return 畸变模型参数
     */
    [[nodiscard]] std::string print() const;

private:
    Float k1_;
    Float k2_;
    Float p1_;
    Float p2_;
};

} // namespace lvins
