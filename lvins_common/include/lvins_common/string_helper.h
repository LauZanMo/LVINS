#pragma once

#include <spdlog/fmt/fmt.h>
#include <spdlog/fmt/ostr.h>

#define LVINS_FORMATTER fmt::formatter

using LVINS_FORMAT_PARSE_CONTEXT = fmt::format_parse_context;
using LVINS_FORMAT_CONTEXT       = fmt::format_context;

template<typename... Args>
auto LVINS_FORMAT(Args &&...args) {
    return fmt::format(std::forward<Args>(args)...);
}

template<typename... Args>
auto LVINS_FORMAT_TO(Args &&...args) {
    return fmt::format_to(std::forward<Args>(args)...);
}

template<typename... Args>
auto LVINS_GROUP_DIGITS(Args &&...args) {
    return fmt::group_digits(std::forward<Args>(args)...);
}

template<typename... Args>
auto LVINS_JOIN(Args &&...args) {
    return fmt::join(std::forward<Args>(args)...);
}
