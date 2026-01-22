#include "AbstractArtnetHandler.hpp"

#include <cmath>
#include <esp_log.h>

static const char *TAG = "AbstractArtnetHandler";

AbstractArtnetHandler::AbstractArtnetHandler(BlockingRingBuffer<PixelFrame> &frameQueue, int pixelCount)
    : PIXEL_COUNT_(pixelCount), UNIVERSE_COUNT_(std::ceil((pixelCount * 3) / static_cast<float>(512))),
      artnetQueue_(frameQueue), artnetFrame_(pixelCount) {
}

void AbstractArtnetHandler::onDmxFrame(uint16_t universeIndex, uint16_t length, uint8_t sequence, uint8_t *data) {
    if (universeIndex >= UNIVERSE_COUNT_) {
        return;
    }

    // Reset frame when universe 0 is received again, since this must belong to the next frame
    if (universeIndex == 0 && !receivedUniverses_.empty() &&
        receivedUniverses_.size() != static_cast<size_t>(UNIVERSE_COUNT_)) {
        receivedUniverses_.clear();
        resetCount_++;
    }

    if (receivedUniverses_.find(universeIndex) != receivedUniverses_.end()) {
        receivedUniverses_.clear();
        resetCount_++;
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

        if (resetCount_ > 0) {
            ESP_LOGW(TAG, "Resetted frame %d time(s) since the last frame", resetCount_);
            resetCount_ = 0;
        }
    }
}

void AbstractArtnetHandler::setChannel(uint16_t universeIndex, int channelIndex, uint8_t value) {
    // integer division will round off to nearest neighbor;
    unsigned pixelIndex = (universeIndex * 512 + channelIndex) / 3;
    // 0 -> first channel, 1 -> second channel, 2 -> third channel
    unsigned rgbChannelIndex = (universeIndex * 512 + channelIndex) % 3;
    if (pixelIndex >= PIXEL_COUNT_) {
        return;
    }
    artnetFrame_[pixelIndex][rgbChannelIndex] = value;
}
