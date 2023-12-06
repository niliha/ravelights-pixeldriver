#pragma once

#include "AbstractPixelHandler.hpp"

#include  <LedControl.h>

#include "PixelFrame.hpp"

class LaserCageHandler : public AbstractPixelHandler{
 public:
    LaserCageHandler(LedControl &ledControl, int laserCount);
    virtual void write(const PixelFrame &frame) override;
    void testLasers();

 private:
    const int laserCount_;

    LedControl &ledControl_;
};