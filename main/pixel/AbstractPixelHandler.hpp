#pragma once

#include "PixelFrame.hpp"

#include <cstdint>
#include <span>

class AbstractPixelHandler {
 public:
    explicit AbstractPixelHandler(uint8_t maxBrightness = 255) : maxBrightness_(maxBrightness) {
    }
    virtual ~AbstractPixelHandler() = default;

    virtual size_t pixelCount() const = 0;
    uint8_t maxBrightness() const { return maxBrightness_; }
    virtual void write(std::span<const Pixel> pixels) = 0;

 private:
    uint8_t maxBrightness_;
};
