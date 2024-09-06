#pragma once

#include "PixelFrame.hpp"

class AbstractPixelHandler {
 public:
    virtual ~AbstractPixelHandler() = default;
    virtual void write(const PixelFrame &frame) = 0;
};