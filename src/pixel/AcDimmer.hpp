#pragma once

#include "PixelFrame.hpp"

namespace AcDimmer {

extern int zeroCrossingCounter;
extern int receiveDelayMicros;

void init(const std::vector<int> triacPins, const int zeroCrossingPin);
void write(const PixelFrame &frame);

}  // namespace AcDimmer
