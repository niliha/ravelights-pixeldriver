#pragma once

#include <optional>
#include <string>

#include "OutputConfig.hpp"

namespace PersistentStorage {

OutputConfig loadOrStoreFallbackOutputConfig(const OutputConfig &fallbackOutputConfig);
std::optional<OutputConfig> loadOutputConfig();
bool storeOutputConfig(const OutputConfig &newOutputConfig);

std::string loadOrStoreFallbackInstanceId(const std::string fallbackInstanceId);
std::optional<std::string> loadInstanceId();
bool storeInstanceId(const std::string &newInstanceId);

void clear();
}  // namespace PersistentStorage
