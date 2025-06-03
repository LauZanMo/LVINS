#include "lvins_common/rotation_helper.h"

namespace lvins::rotation_helper {

template Sophus::SO3f rotation_helper::toSO3(const Eigen::Vector3f &euler);
template Sophus::SO3d rotation_helper::toSO3(const Eigen::Vector3d &euler);

template Eigen::Vector3f rotation_helper::toEuler(const Sophus::SO3f &so3);
template Eigen::Vector3d rotation_helper::toEuler(const Sophus::SO3d &so3);

} // namespace lvins::rotation_helper
