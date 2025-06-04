#include "lvins_icp/preprocess/copy.h"

namespace lvins {

void copy(const RawPointCloud &src, PointCloud &output) {
    if (src.points) {
        output.add_points(src.points, static_cast<int>(src.size()));
    }

    if (src.times) {
        output.add_times(src.times, static_cast<int>(src.size()));
    }

    if (src.normals) {
        output.add_normals(src.normals, static_cast<int>(src.size()));
    }

    if (src.covs) {
        output.add_covs(src.covs, static_cast<int>(src.size()));
    }

    if (src.intensities) {
        output.add_intensities(src.intensities, static_cast<int>(src.size()));
    }

    for (const auto &[name, elem]: src.aux_attributes) {
        const size_t elem_size = elem.first;
        const auto data_ptr    = static_cast<const unsigned char *>(elem.second);

        const auto storage = std::make_shared<std::vector<unsigned char>>(src.size() * elem_size);
        memcpy(storage->data(), data_ptr, elem_size * src.size());

        output.aux_attributes_storage[name] = storage;
        output.aux_attributes[name]         = std::make_pair(elem_size, storage->data());
    }
}

} // namespace lvins
