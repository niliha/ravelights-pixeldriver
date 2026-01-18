#pragma once

#include <FastLED.h>

#include "PixelFrame.hpp"
#include "pixel/AbstractPixelHandler.hpp"

#include <cstdint>
#include <vector>

#ifndef CLOCKLESS_CHIPSET
#define CLOCKLESS_CHIPSET WS2812
#endif

template <const std::array<int, OutputConfig::OUTPUT_COUNT> &Pins, EOrder RgbOrder = RGB>
class FastLedHandler : public AbstractPixelHandler {
 public:
    FastLedHandler(const OutputConfig &pixelsPerOutput)
        : pixelCount_(pixelsPerOutput.getPixelCount()), fastLedPixels_(pixelCount_) {
        int pixelOffset = 0;
        pixelOffset += addLedsOnPin<Pins[0]>(pixelsPerOutput[0], pixelOffset);
        pixelOffset += addLedsOnPin<Pins[1]>(pixelsPerOutput[1], pixelOffset);
        pixelOffset += addLedsOnPin<Pins[2]>(pixelsPerOutput[2], pixelOffset);
        pixelOffset += addLedsOnPin<Pins[3]>(pixelsPerOutput[3], pixelOffset);
    }

    virtual void write(const PixelFrame &frame) override {
        assert(frame.size() == fastLedPixels_.size());
        std::transform(frame.begin(), frame.end(), fastLedPixels_.begin(),
                       [](Pixel pixel) { return CRGB(pixel.r, pixel.g, pixel.b); });

        FastLED.show();
    }

    void testPixels() {
        ESP_LOGI(TAG, "Testing Pixels...");
        for (const auto color : std::vector<CRGB>{CRGB::DarkRed, CRGB::DarkGreen, CRGB::DarkBlue}) {
            auto millisBefore = millis();
            FastLED.showColor(color);
            auto passedMillis = millis() - millisBefore;
            ESP_LOGD(TAG, "show() took %lu ms", passedMillis);
            delay(500);

            FastLED.clear(true);
            delay(500);
        }
    }

 private:
    static constexpr const char *TAG = "FastLedHandler";
    // Total number of pixels across all lights.
    const int pixelCount_;

    // The vector holding color values for each pixel.
    std::vector<CRGB> fastLedPixels_;

    template <int PinNumber> int addLedsOnPin(uint32_t pixelCount, int pixelOffset) {
        if (pixelCount > 0) {
            FastLED.addLeds<CLOCKLESS_CHIPSET, PinNumber, RgbOrder>(fastLedPixels_.data(), pixelOffset, pixelCount);
        }
        return static_cast<int>(pixelCount);
    }
};
