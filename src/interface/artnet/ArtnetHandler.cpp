#include <numeric>

#include "ArtnetHandler.hpp"

ArtnetHandler::ArtnetHandler(BlockingRingBuffer<PixelFrame> &frameQueue, int pixelCount, Mode artnetMode, int baudrate)
    : PIXEL_COUNT_(pixelCount), UNIVERSE_COUNT_(std::ceil((pixelCount * 3) / static_cast<float>(512))),
      artnetQueue_(frameQueue), artnetFrame_(pixelCount) {

    if (artnetMode == Mode::WIFI_ONLY || artnetMode == Mode::WIFI_AND_SERIAL) {
        artnetWifi_ = std::make_unique<ArtnetWifi>();
    }
    if (artnetMode == Mode::SERIAL_ONLY || artnetMode == Mode::WIFI_AND_SERIAL) {
        artnetSerial_ = std::make_unique<ArtnetSerial>(baudrate);
    }

    init();
}

void ArtnetHandler::start() {
    if (artnetWifi_ != nullptr) {
        artnetWifi_->begin();
    }
}

void ArtnetHandler::handleReceived() {
    // These functions call onDmxFrame() whenever a ArtDMX packet is received
    if (artnetWifi_ != nullptr) {
        artnetWifi_->read();
    }

    if (artnetSerial_ != nullptr) {
        artnetSerial_->read();
    }
}

void ArtnetHandler::init() {
    if (artnetWifi_ != nullptr) {
        artnetWifi_->setArtDmxFunc([this](uint16_t universeIndex, uint16_t length, uint8_t sequence, uint8_t *data) {
            this->onDmxFrame(universeIndex, length, sequence, data);
        });
        WiFi.setSleep(false);  // Improves UDP throughput drastically
    }

    if (artnetSerial_ != nullptr) {
        artnetSerial_->setArtDmxCallback(
            [this](uint16_t universeIndex, uint16_t length, uint8_t sequence, uint8_t *data) {
                this->onDmxFrame(universeIndex, length, sequence, data);
            });
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

void ArtnetHandler::onDmxFrame(uint16_t universeIndex, uint16_t length, uint8_t sequence, uint8_t *data) {
    if (universeIndex >= UNIVERSE_COUNT_) {
        return;
    }

    if (receivedUniverses_.find(universeIndex) != receivedUniverses_.end()) {
        Serial.printf("WARN: Received duplicate universe %d\n", universeIndex);
        return;
    }

    receivedUniverses_.insert(universeIndex);

    // Read universe and put into the right part of the display buffer
    for (unsigned channelIndex = 0; channelIndex < length; channelIndex++) {
        setChannel(universeIndex, channelIndex, data[channelIndex]);
    }

    // All data for this frame has been received
    if (receivedUniverses_.size() == UNIVERSE_COUNT_) {
        // Put the filled frame into the queue such that the FasLED task can consume it
        artnetQueue_.push(std::move(artnetFrame_));
        // Create a new vector for the next frame.
        artnetFrame_ = PixelFrame(PIXEL_COUNT_);
        // Reset information about received universes
        receivedUniverses_.clear();
    }
}
