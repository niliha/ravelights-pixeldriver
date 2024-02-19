#pragma once

#include "PixelFrame.hpp"

namespace AcDimmer {

extern int zeroCrossingCounter;

void init(const std::vector<int> triacPins, const int zeroCrossingPin);
void write(const PixelFrame &frame);

}  // namespace AcDimmer
