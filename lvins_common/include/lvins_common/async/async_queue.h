#pragma once

#include "lvins_common/async/mutex_types.h"
#include "lvins_common/non_copyable.h"

#include <condition_variable>
#include <memory>
#include <queue>

namespace lvins {

/**
 * @brief 异步队列
 * @tparam T 队列元素类型
 */
template<typename T>
class AsyncQueue : public NonCopyable {
public:
    using sPtr = std::shared_ptr<AsyncQueue>;

    AsyncQueue() = default;

    /**
     * @brief 将实例压入队列
     * @param obj 实例
     */
    void push(const T &obj) {
        lock_t lock(mutex_);
        cv_pop_.wait(lock, [this] {
            return queue_.size() < capacity_;
        });
        queue_.push(obj);
        cv_push_.notify_one();
    }

    /**
     * @brief 将实例压入队列
     * @param obj 实例
     */
    void push(T &&obj) {
        lock_t lock(mutex_);
        cv_pop_.wait(lock, [this] {
            return queue_.size() < capacity_;
        });
        queue_.push(std::move(obj));
        cv_push_.notify_one();
    }

    /**
     * @brief 从队列中弹出实例
     * @return 弹出的实例
     */
    [[nodiscard]] T pop() {
        lock_t lock(mutex_);
        cv_push_.wait(lock, [this] {
            return !queue_.empty();
        });
        T obj = std::move(queue_.front());
        queue_.pop();
        cv_pop_.notify_one();
        return obj;
    }

    /**
     * @brief 尝试从队列中弹出实例
     * @param obj 弹出的实例
     * @return 是否成功弹出实例
     */
    [[nodiscard]] bool tryPop(T &obj) {
        lock_t lock(mutex_);
        if (queue_.empty()) {
            return false;
        }
        obj = std::move(queue_.front());
        queue_.pop();
        cv_pop_.notify_one();
        return true;
    }

    /**
     * @brief 判断队列是否为空
     * @return 队列是否为空
     */
    [[nodiscard]] bool empty() const {
        lock_t lock(mutex_);
        return queue_.empty();
    }

    /**
     * @brief 清空队列
     */
    void clear() {
        lock_t lock(mutex_);
        std::queue<T> queue;
        std::swap(queue, queue_);
        cv_pop_.notify_all();
    }

    /**
     * @brief 设置队列容量
     * @param capacity 队列容量
     */
    void setCapacity(size_t capacity) {
        capacity_ = capacity;
    }

private:
    mutable mutex_t mutex_;
    std::queue<T> queue_;
    std::condition_variable cv_push_, cv_pop_;
    size_t capacity_ = std::numeric_limits<size_t>::max();
};

} // namespace lvins
