#pragma once

#include "lvins_common/mutex_types.h"

#include <functional>
#include <memory>
#include <tuple>

namespace lvins {

/**
 * @brief 异步实例服务器类
 * @details 异步实例服务器类用于管理多线程对实例集合的读写操作
 * @tparam ObjectTypes 实例类型集合
 */
template<typename... ObjectTypes>
class AsyncServer {
public:
    using sPtr      = std::shared_ptr<AsyncServer>;
    using ReadTask  = std::function<void(const std::shared_ptr<const ObjectTypes> &...)>;
    using WriteTask = std::function<void(const std::shared_ptr<ObjectTypes> &...)>;

    /**
     * @brief 构造函数
     */
    explicit AsyncServer(std::shared_ptr<ObjectTypes>... objects)
        : const_objects_(objects...), objects_(std::move(objects)...) {}

    /**
     * @brief 添加实例读取任务
     * @param task 实例读取任务
     */
    void addReadTask(const ReadTask &task) const {
        rlock_t lock(mutex_);
        std::apply(task, const_objects_);
    }

    /**
     * @brief 添加实例写入任务
     * @param task 实例写入任务
     */
    void addWriteTask(const WriteTask &task) {
        wlock_t lock(mutex_);
        std::apply(task, objects_);
    }

private:
    mutable rw_mutex_t mutex_;                                        ///< 实例读写锁
    std::tuple<std::shared_ptr<const ObjectTypes>...> const_objects_; ///< 常量实例集合
    std::tuple<std::shared_ptr<ObjectTypes>...> objects_;             ///< 实例集合
};

} // namespace lvins
