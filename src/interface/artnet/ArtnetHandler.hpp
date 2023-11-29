#pragma once

#include <ArtnetWifi.h>
#include <memory>
#include <set>
#include <vector>

#include "ArtnetSerial.hpp"
#include "BlockingRingBuffer.hpp"
#include "PixelFrame.hpp"
#include "interface/AbstractInterfaceHandler.hpp"

class ArtnetHandler : public AbstractInterfaceHandler {
 public:
    enum class Mode { WIFI_ONLY, SERIAL_ONLY, WIFI_AND_SERIAL };

    ArtnetHandler(BlockingRingBuffer<PixelFrame> &frameQueue, int pixelCount, Mode artnetMode, int baudrate = 3000000);

    virtual void start() override;
    virtual void handleReceived() override;

 private:
    const int PIXEL_COUNT_;
    const int UNIVERSE_COUNT_;

    BlockingRingBuffer<PixelFrame> &artnetQueue_;
    std::unique_ptr<ArtnetWifi> artnetWifi_;
    std::unique_ptr<ArtnetSerial> artnetSerial_;
    PixelFrame artnetFrame_;
    std::set<int> receivedUniverses_;

    void onDmxFrame(uint16_t universeIndex, uint16_t length, uint8_t sequence, uint8_t *data);
    void setChannel(uint16_t universeIndex, int channelIndex, uint8_t value);
    void init();
};