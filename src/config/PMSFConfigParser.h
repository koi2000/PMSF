/*
 * @file PMSFConfigParser.h
 * @brief Phase A/B: JSON 配置解析器。
 */
#pragma once

#include "PMSFConfig.h"
#include <string>

namespace pmsf {

/// @brief 从 JSON 文件路径解析 PMSFConfig。
/// @throws std::runtime_error 当文件不存在或字段非法时。
PMSFConfig parseConfigFromFile(const std::string& path);

/// @brief 从 JSON 字符串解析 PMSFConfig（方便单元测试）。
PMSFConfig parseConfigFromString(const std::string& json);

}  // namespace pmsf
