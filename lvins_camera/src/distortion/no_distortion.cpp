#include "lvins_camera/distortion/no_distortion.h"

namespace lvins {

Vec2f NoDistortion::distort(const Vec2f &uv) {
    return uv;
}

Vec2f NoDistortion::undistort(const Vec2f &uv) {
    return uv;
}

Mat22f NoDistortion::jacobian(const Vec2f & /*uv*/) {
    return Mat22f::Identity();
}

VecXf NoDistortion::distortionParameters() {
    return {};
}

std::string NoDistortion::print() const {
    return "  Distortion: None";
}

} // namespace lvins
