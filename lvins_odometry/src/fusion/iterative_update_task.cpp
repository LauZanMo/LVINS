#include "lvins_odometry/fusion/iterative_update_task.h"

namespace lvins::eskf {

IterativeUpdateTask::IterativeUpdateTask(int64_t timestamp) : timestamp_(timestamp) {}

int64_t IterativeUpdateTask::timestamp() const {
    return timestamp_;
}

bool IterativeUpdateTask::operator<(const IterativeUpdateTask &other) const {
    return timestamp() < other.timestamp();
}

bool IterativeUpdateTask::operator>(const IterativeUpdateTask &other) const {
    return timestamp() > other.timestamp();
}

} // namespace lvins::eskf
