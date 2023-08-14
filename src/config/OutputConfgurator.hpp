#pragma once

#include "artnet/PixelOutputConfig.hpp"

namespace OutputConfigurator {

PixelOutputConfig load(const PixelOutputConfig &fallbackOutputConfig);
void apply(const PixelOutputConfig &outputConfig);

}  // namespace OutputConfigurator
