#pragma once

#include "lvins_common/async/mutex_types.h"
#include "lvins_common/non_copyable.h"

#include <condition_variable>
#include <memory>
#include <queue>

namespace lvins {

/**
 * @brief 异步优先队列
 * @tparam T 优先队列元素类型
 * @tparam Compare 优先队列比较器
 */
template<typename T, typename Compare = std::less<T>>
class AsyncPriorityQueue : public NonCopyable {
public:
    /**
     * @brief 默认构造函数
     */
    AsyncPriorityQueue() = default;

    /**
     * @brief 将实例压入优先队列
     * @param obj 实例
     */
    void push(const T &obj) {
        lock_t lock(mutex_);
        cv_pop_.wait(lock, [this] {
            return priority_queue_.size() < capacity_;
        });
        priority_queue_.push(obj);
        cv_push_.notify_one();
    }

    /**
     * @brief 将实例压入优先队列
     * @param obj 实例
     */
    void push(T &&obj) {
        lock_t lock(mutex_);
        cv_pop_.wait(lock, [this] {
            return priority_queue_.size() < capacity_;
        });
        priority_queue_.push(std::move(obj));
        cv_push_.notify_one();
    }

    /**
     * @brief 从优先队列中弹出实例
     * @return 弹出的实例
     */
    [[nodiscard]] T pop() {
        lock_t lock(mutex_);
        cv_push_.wait(lock, [this] {
            return !priority_queue_.empty();
        });
        T obj = std::move(priority_queue_.top());
        priority_queue_.pop();
        cv_pop_.notify_one();
        return obj;
    }

    /**
     * @brief 尝试从优先队列中弹出实例
     * @param obj 弹出的实例
     * @return 是否成功弹出实例
     */
    [[nodiscard]] bool tryPop(T &obj) {
        lock_t lock(mutex_);
        if (priority_queue_.empty()) {
            return false;
        }
        obj = std::move(priority_queue_.top());
        priority_queue_.pop();
        cv_pop_.notify_one();
        return true;
    }

    /**
     * @brief 判断优先队列是否为空
     * @return 优先队列是否为空
     */
    [[nodiscard]] bool empty() const {
        lock_t lock(mutex_);
        return priority_queue_.empty();
    }

    /**
     * @brief 清空优先队列
     */
    void clear() {
        lock_t lock(mutex_);
        std::priority_queue<T, std::vector<T>, Compare> queue;
        std::swap(queue, priority_queue_);
        cv_pop_.notify_all();
    }

    /**
     * @brief 设置优先队列容量
     * @param capacity 优先队列容量
     */
    void setCapacity(size_t capacity) {
        capacity_ = capacity;
    }

private:
    mutable mutex_t mutex_;
    std::priority_queue<T, std::vector<T>, Compare> priority_queue_;
    std::condition_variable cv_push_, cv_pop_;
    size_t capacity_ = std::numeric_limits<size_t>::max();
};

} // namespace lvins
