#pragma once

#include <memory>
#include <set>
#include <vector>

#include "BlockingRingBuffer.hpp"
#include "PixelFrame.hpp"
#include "interface/AbstractInterfaceHandler.hpp"

class AbstractArtnetHandler : public AbstractInterfaceHandler {
 public:
    AbstractArtnetHandler(BlockingRingBuffer<PixelFrame> &frameQueue, int pixelCount);

 protected:
    const int PIXEL_COUNT_;
    const int UNIVERSE_COUNT_;

    void onDmxFrame(uint16_t universeIndex, uint16_t length, uint8_t sequence, uint8_t *data);

 private:
    BlockingRingBuffer<PixelFrame> &artnetQueue_;
    PixelFrame artnetFrame_;
    std::set<int> receivedUniverses_;
    int duplicateUniverseCount_ = 0;

    void setChannel(uint16_t universeIndex, int channelIndex, uint8_t value);
};