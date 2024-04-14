#pragma once

#include "AbstractPixelHandler.hpp"

class AcDimmerHandler : public AbstractPixelHandler {
 public:
    AcDimmerHandler(const int pixelCount, const int zeroCrossingPin, const int triacTaskCore);
    virtual void write(const PixelFrame &frame) override;

    void testLights();
};