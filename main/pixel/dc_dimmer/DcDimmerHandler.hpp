#pragma once

#include "pixel/AbstractPixelHandler.hpp"

#include <Adafruit_TLC59711.h>

#include "PixelFrame.hpp"

class DcDimmerHandler : public AbstractPixelHandler {
 public:
    DcDimmerHandler(Adafruit_TLC59711 &tlc59711, const int lightCount, const uint16_t minPwmValue = 0,
                    const uint16_t maxPwmValue = UINT16_MAX / 2);
    virtual void write(const PixelFrame &frame) override;
    void testLights();

 private:
    const int lightCount_;
    const uint16_t minPwmValue_;
    const uint16_t maxPwmValue_;

    Adafruit_TLC59711 &tlc59711_;
};