#pragma once

#include "pixel/AbstractPixelHandler.hpp"

#include <Adafruit_TLC59711.h>

#include "PixelFrame.hpp"

class DcDimmerHandler : public AbstractPixelHandler {
 public:
    DcDimmerHandler(Adafruit_TLC59711 &tlc59711, const int lightCount);
    virtual void write(const PixelFrame &frame) override;
    void testLights();

 private:
    const int lightCount_;

    Adafruit_TLC59711 &tlc59711_;
};