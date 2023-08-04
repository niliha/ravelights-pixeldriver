#pragma once

#include <array>

struct PixelConfig {
    std::array<uint8_t, 4> lightsPerOutput;

    PixelConfig(std::array<uint8_t,4> lightsPerOutput) {
        this->lightsPerOutput = lightsPerOutput;
    }
};