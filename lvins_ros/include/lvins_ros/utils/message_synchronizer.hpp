#include "lvins_common/logger.h"

#include <rclcpp/rclcpp.hpp>

namespace lvins {

template<typename T>
MessageSynchronizer<T>::MessageSynchronizer(size_t sync_size) : messages_(sync_size) {
    match_flag_.reset();
}

template<typename T>
bool MessageSynchronizer<T>::push(size_t idx, const MessagePtr &msg) {
    const auto timestamp = rclcpp::Time(msg->header.stamp).nanoseconds();

    if (match_flag_.none()) {
        ref_timestamp  = timestamp;
        messages_[idx] = msg;
        match_flag_.set(idx);
    } else {
        if (timestamp == ref_timestamp) {
            messages_[idx] = msg;
            match_flag_.set(idx);
        } else if (timestamp > ref_timestamp) {
            LVINS_WARN("Message [{}] time jump found at {}", idx, LVINS_GROUP_DIGITS(ref_timestamp));
            ref_timestamp  = timestamp;
            messages_[idx] = msg;
            match_flag_.reset();
            match_flag_.set(idx);
        }
    }

    if (match_flag_.count() == messages_.size()) {
        match_flag_.reset();
        return true;
    }

    return false;
}

template<typename T>
const std::vector<typename MessageSynchronizer<T>::MessagePtr> &MessageSynchronizer<T>::getSyncMessages() const {
    return messages_;
}

} // namespace lvins
