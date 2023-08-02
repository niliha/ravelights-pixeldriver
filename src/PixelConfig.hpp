#pragma once

#include <array>

struct PixelConfig {
    std::array<int, 4> lightsPerOutput;

    PixelConfig(std::array<int,4> lightsPerOutput) {
        this->lightsPerOutput = lightsPerOutput;
    }
};