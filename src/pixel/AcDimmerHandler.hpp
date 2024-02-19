#pragma once

#include "AbstractPixelHandler.hpp"

class AcDimmerHandler : public AbstractPixelHandler {
 public:
    AcDimmerHandler(const std::vector<int> triacPins, const int zeroCrossingPin);
    virtual void write(const PixelFrame &frame) override;
};