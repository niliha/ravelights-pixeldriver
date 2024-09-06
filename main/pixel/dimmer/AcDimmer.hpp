#pragma once

#include "PixelFrame.hpp"

namespace AcDimmer {

void init(const int channelCount, const int zeroCrossingPin, const int triacTaskCore = 1);
void write(const PixelFrame &frame);
void testLights();

}  // namespace AcDimmer
