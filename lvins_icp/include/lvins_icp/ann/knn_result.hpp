#include "lvins_common/logger.h"

#include <limits>

namespace lvins {

template<typename IndexTransform>
KnnResult<IndexTransform>::KnnResult(std::vector<size_t> &indices, std::vector<Float> &sq_dists, size_t capacity,
                                     Float max_sq_dist, const IndexTransform &index_transform)
    : indices_(indices), sq_dists_(sq_dists), capacity_(capacity), index_transform_(index_transform) {
    LVINS_CHECK(capacity_ > 0, "Capacity should be greater than 0!");
    LVINS_CHECK(max_sq_dist > 0, "Max squared distance should be greater than 0!");

    // 容器初始化
    indices_.clear();
    sq_dists_.clear();
    indices_.resize(capacity_, std::numeric_limits<size_t>::max());
    sq_dists_.resize(capacity_, max_sq_dist);
}

template<typename IndexTransform>
size_t KnnResult<IndexTransform>::capacity() const {
    return capacity_;
}

template<typename IndexTransform>
size_t KnnResult<IndexTransform>::numFound() const {
    return num_found_;
}

template<typename IndexTransform>
void KnnResult<IndexTransform>::push(size_t index, Float sq_dist) {
    if (sq_dist >= sq_dists_.back()) {
        return;
    }

    // 倒序查找与移动
    auto insert_loc = static_cast<int>(std::min<size_t>(num_found_, capacity_ - 1));
    for (; insert_loc > 0 && sq_dist < sq_dists_[insert_loc - 1]; --insert_loc) {
        indices_[insert_loc]  = indices_[insert_loc - 1];
        sq_dists_[insert_loc] = sq_dists_[insert_loc - 1];
    }
    indices_[insert_loc]  = index_transform_(index);
    sq_dists_[insert_loc] = sq_dist;

    num_found_ = std::min<size_t>(num_found_ + 1, capacity_);
}

} // namespace lvins
