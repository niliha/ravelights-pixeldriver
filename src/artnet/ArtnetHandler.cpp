#include <numeric>

#include "ArtnetHandler.hpp"

ArtnetHandler::ArtnetHandler(BlockingRingBuffer<std::variant<PixelFrame, PixelOutputConfig>> &frameQueue,
                             int pixelCount, int baudrate)
    : PIXEL_COUNT_(pixelCount), UNIVERSE_COUNT_(std::ceil((pixelCount * 3) / static_cast<float>(512))),
      artnetQueue_(frameQueue), artnetSerial_(baudrate), artnetFrame_(pixelCount) {
    // Improves UDP throughput drastically
    WiFi.setSleep(false);

    setArtnetCallback();
    artnetWifi_.begin();
}

void ArtnetHandler::read() {
    // These functions call onDmxFrame() whenever a ArtDMX packet is received
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

void ArtnetHandler::handleConfig(uint16_t length, uint8_t *dataBytes) {
    if (length != CONFIG_UNIVERSE_LENGTH) {
        Serial.printf("ERROR: Expected %d config channels but got %d\n", CONFIG_UNIVERSE_LENGTH, length);
        return;
    }

    uint32_t *dataInts = reinterpret_cast<uint32_t *>(dataBytes);
    PixelOutputConfig pixelsPerOutput = {dataInts[0], dataInts[1], dataInts[2], dataInts[3]};

    uint32_t checksum = std::accumulate(pixelsPerOutput.begin(), pixelsPerOutput.end(), 0);
    if (checksum != dataInts[4]) {
        Serial.printf("ERROR: Calculated config checksum %d does not match with %d\n", checksum, dataBytes[4]);
        return;
    }

    artnetQueue_.push(std::move(pixelsPerOutput));
    return;
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
    if (universeIndex == CONFIG_UNIVERSE_INDEX) {
        handleConfig(length, data);
        return;
    }

    if (universeIndex >= UNIVERSE_COUNT_) {
        return;
    }

    if (receivedUniverses_.find(universeIndex) != receivedUniverses_.end()) {
        Serial.printf("WARN: Received duplicate universe %d\n", universeIndex);
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
        // Put the filled frame into the queue such that the FasLED task can consume it
        artnetQueue_.push(std::move(artnetFrame_));
        // Create a new vector for the next frame.
        artnetFrame_ = PixelFrame(PIXEL_COUNT_);
        // Reset information about received universes
        receivedUniverses_.clear();
    }
}
