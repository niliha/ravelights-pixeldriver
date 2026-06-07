#pragma once

#include "AbstractPixelHandler.hpp"

#include <Tlc5940.h>

#include "PixelFrame.hpp"

class LaserCageHandler : public AbstractPixelHandler {
 public:
    LaserCageHandler(int laserCount, int sin, int sclk, int xlat, int blank, int gsclk, uint8_t maxBrightness = 255);
    size_t pixelCount() const override { return laserCount_; }
    virtual void write(std::span<const Pixel> pixels) override;
    void testLasers();

 private:
    const int laserCount_;
};
