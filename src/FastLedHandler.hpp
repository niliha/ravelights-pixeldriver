#pragma once

#define FASTLED_ESP32_I2S  // Alternative parallel output driver
#include <FastLED.h>

#include <numeric>
#include <vector>

#include "BlockingRingBuffer.hpp"

template <const std::array<int, 4> &PINS, EOrder RGB_ORDER = RGB> class FastLedHandler {
 public:
    FastLedHandler(const PixelOutputConfig &pixelsPerOutput)
        : PIXEL_COUNT_(std::accumulate(pixelsPerOutput.begin(), pixelsPerOutput.end(), 0)), fastLedPixels_(PIXEL_COUNT_)

    {
        setupFastled(pixelsPerOutput);
    }

    void write(const std::vector<CRGB> &frame) {
        assert(frame.size() == fastLedPixels_.size());

        fastLedPixels_ = frame;
        FastLED.show();
    }

    void testLeds() {
        Serial.println("Testing LEDs...");
        for (const auto color : std::vector<CRGB>{CRGB::Red, CRGB::Green, CRGB::Blue}) {
            auto millisBefore = millis();
            FastLED.showColor(color);
            auto passedMillis = millis() - millisBefore;
            Serial.printf("show() took %d ms\n", passedMillis);
            delay(500);
            FastLED.clear(true);
            delay(500);
        }
        delay(1000);
    }

    int getPixelCount() const {
        return PIXEL_COUNT_;
    }

 private:
    // Total number of pixels across all lights.
    const int PIXEL_COUNT_;

    // The vector holding color values for each pixel.
    std::vector<CRGB> fastLedPixels_;

    void setupFastled(const PixelOutputConfig &pixelsPerOutput) {
        // We can't use a loop here since addLeds() template parameters must be known at
        // compile-time. With c++20 a constexpr for loop can be used.
        int pixelOffset = 0;
        if (pixelsPerOutput[0] > 0) {
            FastLED.addLeds<WS2812, PINS[0], RGB_ORDER>(fastLedPixels_.data(), pixelOffset, pixelsPerOutput[0]);
            pixelOffset += pixelsPerOutput[0];
        }
        if (pixelsPerOutput[1] > 0) {
            FastLED.addLeds<WS2812, PINS[1], RGB_ORDER>(fastLedPixels_.data(), pixelOffset, pixelsPerOutput[1]);
            pixelOffset += pixelsPerOutput[1];
        }
        if (pixelsPerOutput[2] > 0) {
            FastLED.addLeds<WS2812, PINS[2], RGB_ORDER>(fastLedPixels_.data(), pixelOffset, pixelsPerOutput[2]);
            pixelOffset += pixelsPerOutput[2];
        }
        if (pixelsPerOutput[3] > 0) {
            FastLED.addLeds<WS2812, PINS[3], RGB_ORDER>(fastLedPixels_.data(), pixelOffset, pixelsPerOutput[3]);
            pixelOffset += pixelsPerOutput[3];
        }

        FastLED.setCorrection(TypicalLEDStrip);
    }
};
