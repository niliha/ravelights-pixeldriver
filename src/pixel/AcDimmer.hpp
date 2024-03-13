#pragma once

#include "PixelFrame.hpp"

namespace AcDimmer {

void init(const int channelCount, const int zeroCrossingPin);
void write(const PixelFrame &frame);

}  // namespace AcDimmer
