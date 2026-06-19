#pragma once

#include <FastLED.h>

#include "AbstractPixelHandler.hpp"
#include "PixelFrame.hpp"

#include <cstddef>
#include <vector>

#ifndef CLOCKLESS_CHIPSET
#define CLOCKLESS_CHIPSET WS2812
#endif

template <int Pin, EOrder RgbOrder = RGB> class SinglePinFastLedHandler : public AbstractPixelHandler {
 public:
    SinglePinFastLedHandler(size_t pixelCount, uint8_t maxBrightness = 255)
        : AbstractPixelHandler(maxBrightness), pixelCount_(pixelCount), fastLedPixels_(pixelCount) {
        FastLED.addLeds<CLOCKLESS_CHIPSET, Pin, RgbOrder>(fastLedPixels_.data(), 0, pixelCount);
    }

    size_t pixelCount() const override {
        return pixelCount_;
    }

    void write(std::span<const Pixel> pixels) override {
        assert(pixels.size() == pixelCount_);
        std::transform(pixels.begin(), pixels.end(), fastLedPixels_.begin(),
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
    static constexpr const char *TAG = "SinglePinFastLedHandler";
    const size_t pixelCount_;
    std::vector<CRGB> fastLedPixels_;
};
