#include "ArtnetHandler.hpp"

ArtnetHandler::ArtnetHandler(BlockingRingBuffer<std::vector<CRGB>> &frameQueue, int pixelCount, int baudrate)
    : frameQueue_(frameQueue), artnetSerial_(baudrate), PIXEL_COUNT_(pixelCount), artnetFrame_(pixelCount),
      UNIVERSE_COUNT_(std::ceil((pixelCount * 3) / static_cast<float>(512))) {
    // The watchdog on core 0 is not reset anymore, since the idle task is not resumed.
    // It is disabled to avoid watchdog timeouts (resulting in a reboot).
    disableCore0WDT();

    // Improves UDP throughput drastically
    WiFi.setSleep(false);

    setArtnetCallback();
    artnetWifi_.begin();
}

void ArtnetHandler::read() {
    // This calls onDmxFrame() whenever a ArtDMX packet is received
    artnetWifi_.read();
    artnetSerial_.read();
}

void ArtnetHandler::setArtnetCallback() {
    artnetWifi_.setArtDmxFunc([this](uint16_t universeIndex, uint16_t length, uint8_t sequence, uint8_t *data) {
        this->onDmxFrame(universeIndex, length, sequence, data);
    });
    artnetSerial_.setArtDmxCallback([this](uint16_t universeIndex, uint16_t length, uint8_t sequence, uint8_t *data) {
        this->onDmxFrame(universeIndex, length, sequence, data);
    });
}

void ArtnetHandler::onDmxFrame(uint16_t universeIndex, uint16_t length, uint8_t sequence, uint8_t *data) {
    if (universeIndex >= UNIVERSE_COUNT_) {
        return;
    }

    // Store which universe has got in
    receivedUniverses_.insert(universeIndex);

    // Read universe and put into the right part of the display buffer
    for (unsigned channelIndex = 0; channelIndex < length; channelIndex++) {
        setChannel(universeIndex, channelIndex, data[channelIndex]);
    }

    // All data for this frame has been received
    if (receivedUniverses_.size() == UNIVERSE_COUNT_) {
        // Reset information about received universes
        receivedUniverses_.clear();

        // Put the filled frame into the queue such that the FasLED task can consume it
        frameQueue_.push(std::move(artnetFrame_));

        // Create a new vector for the next frame.
        artnetFrame_ = std::vector<CRGB>(PIXEL_COUNT_);
    }
}

void ArtnetHandler::setChannel(uint16_t universeIndex, int channelIndex, uint8_t value) {
    // integer division will round off to nearest neighbor;
    unsigned pixelIndex = (universeIndex * 512 + channelIndex) / 3;
    // 0 -> first channel, 1 -> second channel, 2 -> third channel
    unsigned rgbChannelIndex = (universeIndex * 512 + channelIndex) % 3;
    if (pixelIndex >= PIXEL_COUNT_) {
        return;
    }
    artnetFrame_[pixelIndex][rgbChannelIndex] = value;
}