#pragma once

#include "AbstractPixelHandler.hpp"

class AcDimmerHandler : public AbstractPixelHandler {
 public:
    AcDimmerHandler(const int channels, const int zeroCrossingPin);
    virtual void write(const PixelFrame &frame) override;
};