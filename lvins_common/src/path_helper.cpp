#include "lvins_common/path_helper.h"
#include "lvins_common/logger.h"

namespace lvins::path_helper {

std::string completePath(const std::string &file) {
    // 空路径或绝对路径直接返回
    if (file.empty() || file.front() == '/') {
        return file;
    }

    // 相对路径则补全为绝对路径
    return LVINS_FORMAT("{}/{}", LVINS_CONFIG_DIR, file);
}

std::string getFileName(const std::string &file) {
    const std::string separator = "/";
    const auto last_separator   = file.find_last_of(separator);
    if (last_separator == std::string::npos) {
        return {};
    }
    return file.substr(last_separator + 1);
}

std::string getFilePath(const std::string &file) {
    const std::string separator = "/";
    const auto last_separator   = file.find_last_of(separator);
    if (last_separator == std::string::npos) {
        return {};
    }
    return file.substr(0, last_separator);
}

} // namespace lvins::path_helper
