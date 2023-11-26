#pragma once

#include <ArtnetWifi.h>
#include <memory>
#include <set>
#include <variant>
#include <vector>

#include "artnet/ArtnetSerial.hpp"
#include "artnet/BlockingRingBuffer.hpp"
#include "artnet/PixelOutputConfig.hpp"
#include "common/PixelFrame.hpp"

class ArtnetHandler {
 public:
    enum class Mode { WIFI_ONLY, SERIAL_ONLY, WIFI_AND_SERIAL };

    ArtnetHandler(BlockingRingBuffer<std::variant<PixelFrame, PixelOutputConfig>> &frameQueue, int pixelCount,
                  Mode artnetMode, int baudrate = 3000000);
    void read();

 private:
    static const uint16_t CONFIG_UNIVERSE_INDEX = UINT8_MAX;
    // Pixels per output for 4 pins (uint32_t) + checksum (uint32_t)
    static const uint16_t CONFIG_UNIVERSE_LENGTH = 5 * 4;

    const int PIXEL_COUNT_;
    const int UNIVERSE_COUNT_;

    BlockingRingBuffer<std::variant<PixelFrame, PixelOutputConfig>> &artnetQueue_;
    std::unique_ptr<ArtnetWifi> artnetWifi_;
    std::unique_ptr<ArtnetSerial> artnetSerial_;
    PixelFrame artnetFrame_;
    std::set<int> receivedUniverses_;

    void onDmxFrame(uint16_t universeIndex, uint16_t length, uint8_t sequence, uint8_t *data);
    void setChannel(uint16_t universeIndex, int channelIndex, uint8_t value);
    void handleConfig(uint16_t length, uint8_t *data);
    void init();
};