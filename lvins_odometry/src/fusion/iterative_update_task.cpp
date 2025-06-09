#include "lvins_odometry/fusion/iterative_update_task.h"

namespace lvins::eskf {

IterativeUpdateTask::IterativeUpdateTask(int64_t timestamp) : timestamp_(timestamp) {}

int64_t IterativeUpdateTask::timestamp() const {
    return timestamp_;
}

} // namespace lvins::eskf
