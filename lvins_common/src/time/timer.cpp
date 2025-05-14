#include "lvins_common/time/timer.h"

#include <spdlog/fmt/chrono.h>

namespace lvins {

Timer::Timer() {
    restart();
}

void Timer::restart() {
    start_ = Clock::now();
    stop_  = false;
}

void Timer::stop() {
    duration_ = Clock::now() - start_;
    stop_     = true;
}

double Timer::costInSec() {
    if (!stop_)
        stop();

    using namespace std::chrono;
    return duration_cast<duration<double>>(duration_).count();
}

double Timer::costInMsec() {
    return costInSec() * 1000.0;
}

int64_t Timer::costInNsec() {
    if (!stop_)
        stop();

    return duration_.count();
}

int64_t Timer::currentTimeInMsec() {
    using namespace std::chrono;
    return duration_cast<milliseconds>(Clock::now().time_since_epoch()).count();
}

std::string Timer::currentTime() {
    const auto t = std::time(nullptr);
    return LVINS_FORMAT("{:%Y-%m-%d %H:%M:%S}", fmt::localtime(t));
}

} // namespace lvins