#pragma once

#include "AbstractPixelHandler.hpp"

#include <Tlc5940.h>

#include "PixelFrame.hpp"

class LaserCageHandler : public AbstractPixelHandler {
 public:
    LaserCageHandler(int laserCount, int sin, int sclk, int xlat, int blank, int gsclk);
    virtual void write(const PixelFrame &frame) override;
    void testLasers();

 private:
    const int laserCount_;
};
