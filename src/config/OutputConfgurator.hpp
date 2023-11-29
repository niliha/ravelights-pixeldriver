#pragma once

#include <optional>

#include "PixelOutputConfig.hpp"

namespace OutputConfigurator {

PixelOutputConfig loadOrApplyFallback(const PixelOutputConfig &fallbackOutputConfig);
std::optional<PixelOutputConfig> loadFromFlash();
bool applyToFlash(const PixelOutputConfig &newOutputConfig);
void applyToFlashAndReboot(const PixelOutputConfig &newOutputConfig);

}  // namespace OutputConfigurator
