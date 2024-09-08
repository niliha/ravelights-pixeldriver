#pragma once

#include "AbstractPixelHandler.hpp"

#include <Tlc5940.h>

#include "PixelFrame.hpp"

class LaserCageHandler : public AbstractPixelHandler {
 public:
    /**
     * Connect the TLC5940 to the ESP32 using the following pins:
     * SIN -> G33, (SOUT -> G25), SCLK -> G32, XLAT -> G27, BLANK -> G23, GSCLK -> G12,
     */
    LaserCageHandler(int laserCount);
    virtual void write(const PixelFrame &frame) override;
    void testLasers();

 private:
    const int laserCount_;
};