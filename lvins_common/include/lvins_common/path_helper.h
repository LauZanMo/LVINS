#pragma once

#include <string>

namespace lvins::path_helper {

/**
 * @brief 补全文件路径
 * @details 若为相对路径，则补全为绝对路径，绝对路径则直接返回
 * @param file 需要补全的文件
 * @return 绝对路径
 */
std::string completePath(const std::string &file);

/**
 * @brief 获取文件名
 * @param file 文件（绝对路径或相对路径）
 * @return 文件名
 */
std::string getFileName(const std::string &file);

/**
 * @brief 获取文件路径
 * @param file 文件（绝对路径或相对路径）
 * @return 文件路径
 */
std::string getFilePath(const std::string &file);

} // namespace lvins::path_helper
