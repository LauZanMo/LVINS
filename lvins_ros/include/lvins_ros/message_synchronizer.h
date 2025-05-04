#pragma once

#include "lvins_config/setup.h"

#include <bitset>
#include <memory>
#include <vector>

namespace lvins {

/**
 * @brief 消息同步器
 * @tparam T 消息类型
 */
template<typename T>
class MessageSynchronizer {
public:
    using uPtr       = std::unique_ptr<MessageSynchronizer>;
    using MessagePtr = typename T::ConstSharedPtr;

    /**
     * @brief 构造函数
     * @param sync_size 同步消息数量
     */
    explicit MessageSynchronizer(size_t sync_size);

    /**
     * @brief 将消息加入同步器
     * @param idx 消息索引
     * @param msg 消息
     * @return 是否同步完成
     */
    [[nodiscard]] bool push(size_t idx, const MessagePtr &msg);

    /**
     * @brief 获取已同步消息
     * @return 已同步消息
     */
    [[nodiscard]] const std::vector<MessagePtr> &getSyncMessages() const;

private:
    int64_t ref_timestamp{-1};
    std::vector<MessagePtr> messages_;
    std::bitset<LVINS_MAX_SENSOR_SIZE> match_flag_;
};

} // namespace lvins

#include "lvins_ros/message_synchronizer.hpp"
