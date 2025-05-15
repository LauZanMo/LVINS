#include "lvins_odometry/fusion/update_task.h"

namespace lvins::eskf {

UpdateTask::UpdateTask(int64_t timestamp) : timestamp_(timestamp) {}

int64_t UpdateTask::timestamp() const {
    return timestamp_;
}

bool UpdateTask::operator<(const UpdateTask &other) const {
    return timestamp() < other.timestamp();
}

bool UpdateTask::operator>(const UpdateTask &other) const {
    return timestamp() > other.timestamp();
}

} // namespace lvins::eskf
