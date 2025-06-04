#include "lvins_odometry/fusion/ins_helper.h"
#include "lvins_common/logger.h"

namespace lvins::ins_helper {

bool detectZeroVelocity(const Imus &imus, Float zero_gyr_thresh, Float zero_acc_thresh) {
    LVINS_CHECK(!imus.empty(), "Imus should not be empty!");
    const auto size_inv = LVINS_FLOAT(1.0) / LVINS_FLOAT(imus.size());

    // 计算平均值
    Vec3f ave_gyr(Vec3f::Zero()), ave_acc(Vec3f::Zero());
    for (const auto &imu: imus) {
        ave_gyr += imu.gyr;
        ave_acc += imu.acc;
    }
    ave_gyr *= size_inv;
    ave_acc *= size_inv;

    // 计算标准差
    Vec3f std_gyr(Vec3f::Zero()), std_acc(Vec3f::Zero());
    for (const auto &imu: imus) {
        std_gyr += (imu.gyr - ave_gyr).cwiseAbs2();
        std_acc += (imu.acc - ave_acc).cwiseAbs2();
    }
    std_gyr = (std_gyr * size_inv).cwiseSqrt();
    std_acc = (std_acc * size_inv).cwiseSqrt();

    LVINS_DEBUG("Zero velocity detector:\n"
                "  Gyroscope std: {}\n"
                "  Accelerator std: {}",
                LVINS_VECTOR_FMT(std_gyr), LVINS_VECTOR_FMT(std_acc));

    return (std_gyr.array() < zero_gyr_thresh).all() && (std_acc.array() < zero_acc_thresh).all();
}

} // namespace lvins::ins_helper
