#include "lvins_common/logger.h"

namespace lvins {

template<typename Projection>
CameraGeometry<Projection>::CameraGeometry(std::string label, int width, int height, cv::Mat mask,
                                           std::string mask_file_name, const Projection &projection)
    : label_(std::move(label)),
      width_(width),
      height_(height),
      mask_(std::move(mask)),
      mask_file_name_(std::move(mask_file_name)),
      projection_(projection) {
    LVINS_CHECK(mask_.empty() || (mask_.rows == height_ && mask_.cols == width_),
                "Mask size should match camera size!");
}

template<typename Projection>
bool CameraGeometry<Projection>::project(const Eigen::Ref<const Vec3f> &point, Eigen::Ref<Vec2f> &out_keypoint) const {
    return projection_.project(point, out_keypoint) && isKeypointVisible(out_keypoint);
}

template<typename Projection>
bool CameraGeometry<Projection>::project(const Eigen::Ref<const Vec3f> &point, Eigen::Ref<Vec2f> &out_keypoint,
                                         Mat23f &out_jacobian) const {
    return projection_.project(point, out_keypoint, out_jacobian) && isKeypointVisible(out_keypoint);
}

template<typename Projection>
bool CameraGeometry<Projection>::unproject(const Eigen::Ref<const Vec2f> &keypoint,
                                           Eigen::Ref<Vec3f> &out_point) const {
    return isKeypointVisible(keypoint) && projection_.unproject(keypoint, out_point);
}

template<typename Projection>
bool CameraGeometry<Projection>::unproject(const Eigen::Ref<const Vec2f> &keypoint, Eigen::Ref<Vec3f> &out_point,
                                           Mat32f &out_jacobian) const {
    return isKeypointVisible(keypoint) && projection_.unproject(keypoint, out_point, out_jacobian);
}

template<typename Projection>
int CameraGeometry<Projection>::id() const {
    return id_;
}

template<typename Projection>
void CameraGeometry<Projection>::setId(int id) {
    id_ = id;
}

template<typename Projection>
const std::string &CameraGeometry<Projection>::label() const {
    return label_;
}

template<typename Projection>
int CameraGeometry<Projection>::width() const {
    return width_;
}

template<typename Projection>
int CameraGeometry<Projection>::height() const {
    return height_;
}

template<typename Projection>
bool CameraGeometry<Projection>::hasMask() const {
    return !mask_.empty();
}

template<typename Projection>
const cv::Mat &CameraGeometry<Projection>::mask() const {
    return mask_;
}

template<typename Projection>
bool CameraGeometry<Projection>::isMasked(const Eigen::Ref<const Vec2f> &keypoint) const {
    return mask_.ptr<uint8_t>(static_cast<int>(keypoint[1]))[static_cast<int>(keypoint[0])] == 0;
}

template<typename Projection>
const std::string &CameraGeometry<Projection>::maskFileName() const {
    return mask_file_name_;
}

template<typename Projection>
VecXf CameraGeometry<Projection>::intrinsicParameters() const {
    return projection_.intrinsicParameters();
}

template<typename Projection>
VecXf CameraGeometry<Projection>::distortionParameters() const {
    return projection_.distortionParameters();
}

template<typename Projection>
const Projection &CameraGeometry<Projection>::projection() const {
    return projection_;
}

template<typename Projection>
std::string CameraGeometry<Projection>::print() const {
    return LVINS_FORMAT("{}\n"
                        "  name = {}\n"
                        "  size = [{} x {}]\n"
                        "{}",
                        label_, label_, width_, height_, projection_.print());
}

} // namespace lvins
