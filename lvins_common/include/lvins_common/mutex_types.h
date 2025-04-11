#pragma once

#include <mutex>
#include <shared_mutex>

namespace lvins {

using mutex_t    = std::mutex;
using rw_mutex_t = std::shared_mutex;
using lock_t     = std::unique_lock<mutex_t>;
using rlock_t    = std::shared_lock<rw_mutex_t>;
using wlock_t    = std::unique_lock<rw_mutex_t>;

} // namespace lvins
