#pragma once

#include "lvins_common/eigen_types.h"

namespace lvins {

/**
 * @brief 无畸变的畸变模型类
 * @details 该类为畸变模型的无畸变实现（理想相机）
 */
class NoDistortion {
public:
    /**
     * @brief 构造函数
     */
    NoDistortion() = default;

    /**
     * @brief 析构函数
     */
    ~NoDistortion() = default;

    /**
     * @brief 将无畸变的归一化平面坐标转换为畸变的归一化平面坐标
     * @param uv 无畸变的归一化平面坐标
     * @return 畸变的归一化平面坐标
     */
    [[nodiscard]] static Vec2f distort(const Vec2f &uv);

    /**
     * @brief 将畸变的归一化平面坐标转换为无畸变的归一化平面坐标
     * @param uv 畸变的归一化平面坐标
     * @return 无畸变的归一化平面坐标
     */
    [[nodiscard]] static Vec2f undistort(const Vec2f &uv);

    /**
     * @brief 计算畸变的归一化平面坐标对无畸变的归一化平面坐标的雅可比矩阵
     * @return 畸变的归一化平面坐标对无畸变的归一化平面坐标的雅可比矩阵
     */
    [[nodiscard]] static Mat22f jacobian(const Vec2f &uv);

    /**
     * @brief 获取相机畸变参数
     * @return 包含相机畸变参数的向量
     */
    [[nodiscard]] static VecXf distortionParameters();

    /**
     * @brief 打印畸变模型参数
     * @return 畸变模型参数
     */
    [[nodiscard]] std::string print() const;
};

} // namespace lvins
