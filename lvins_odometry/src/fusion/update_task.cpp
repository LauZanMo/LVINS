#include "lvins_odometry/fusion/update_task.h"

namespace lvins::eskf {

UpdateTask::UpdateTask(int64_t timestamp) : timestamp_(timestamp) {}

int64_t UpdateTask::timestamp() const {
    return timestamp_;
}

} // namespace lvins::eskf
