#pragma once

#include <cstdint>
#include <vector>

struct TriacEvent {
    unsigned long delayMicros;
    std::vector<std::pair<int, bool>> channels;

    TriacEvent(uint32_t delayMicros, std::vector<std::pair<int, bool>> channels)
        : delayMicros(delayMicros), channels(channels) {
    }
};