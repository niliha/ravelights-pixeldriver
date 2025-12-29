#pragma once

#include "pixel/AbstractPixelHandler.hpp"

#include <Adafruit_TLC59711.h>

#include "PixelFrame.hpp"

class DcDimmerHandler : public AbstractPixelHandler {
 public:
    DcDimmerHandler(Adafruit_TLC59711 &tlc59711, const int lightCount, const uint8_t maxBrightness);
    virtual void write(const PixelFrame &frame) override;
    void testLights();

 private:
    const int LIGHT_COUNT_;
    const uint8_t MAX_BRIGHTNESS_;

    Adafruit_TLC59711 &tlc59711_;
};