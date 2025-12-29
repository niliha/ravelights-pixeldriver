#pragma once

#include <cstdint>

class AbstractTriacDriver {
 public:
    virtual ~AbstractTriacDriver() = default;
    virtual void stageChannel(uint16_t channel, bool turnOn) = 0;
    virtual void commitStagedChannels() = 0;
};