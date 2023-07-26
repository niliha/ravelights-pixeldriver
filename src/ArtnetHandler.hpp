#pragma once

#include <ArtnetWifi.h>
#include <FastLED.h>
#include <set>
#include <vector>

#include "ArtnetSerial.hpp"
#include "BlockingRingBuffer.hpp"

class ArtnetHandler {
 public:
    ArtnetHandler(BlockingRingBuffer<std::vector<CRGB>> &frameQueue, int pixelCount, int baudrate = 3000000);
    void read();

 private:
    const int PIXEL_COUNT_;
    const int UNIVERSE_COUNT_;

    BlockingRingBuffer<std::vector<CRGB>> &frameQueue_;
    ArtnetWifi artnetWifi_;
    ArtnetSerial artnetSerial_;
    std::vector<CRGB> artnetFrame_;
    std::set<int> receivedUniverses_;

    void setChannel(uint16_t universeIndex, int channelIndex, uint8_t value);
    void onDmxFrame(uint16_t universeIndex, uint16_t length, uint8_t sequence, uint8_t *data);

    void setArtnetCallback();
};