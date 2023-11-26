#pragma once

#include <optional>

#include "artnet/PixelOutputConfig.hpp"

namespace OutputConfigurator {

PixelOutputConfig loadOrApplyFallback(const PixelOutputConfig &fallbackOutputConfig);
std::optional<PixelOutputConfig> load();
void apply(const PixelOutputConfig &newOutputConfig);
void applyAndReboot(const PixelOutputConfig &newOutputConfig);

}  // namespace OutputConfigurator
