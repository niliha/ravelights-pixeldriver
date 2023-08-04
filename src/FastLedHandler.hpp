#pragma once

#define FASTLED_ESP32_I2S  // Alternative parallel output driver
#include <FastLED.h>

#include <numeric>
#include <vector>

#include "BlockingRingBuffer.hpp"

template <int PIN_COUNT, const std::array<int, PIN_COUNT> &PINS, EOrder RGB_ORDER = RGB> class FastLedHandler {
 public:
    FastLedHandler(const std::array<uint8_t, PIN_COUNT> &lightsPerPin, int pixelsPerLight = 144)
        : PIXELS_PER_LIGHT_(pixelsPerLight),
          PIXEL_COUNT_(PIXELS_PER_LIGHT_ * std::accumulate(lightsPerPin.begin(), lightsPerPin.end(), 0)),
          fastLedPixels_(PIXEL_COUNT_)

    {
        setupFastled(lightsPerPin);
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
    // The number of individually addressable pixels per ravelight
    const int PIXELS_PER_LIGHT_;
    // Total number of pixels across all lights.
    const int PIXEL_COUNT_;

    // The vector holding color values for each pixel.
    std::vector<CRGB> fastLedPixels_;

    void setupFastled(const std::array<uint8_t, PIN_COUNT> &lightsPerPin) {
        static_assert(PIN_COUNT == 4, "setupFastLed() is hardcoded to handle exactly 4 pins!");
        // We can't use a loop here since addLeds() template parameters must be known at
        // compile-time
        int pixelOffset = 0;
        if (lightsPerPin[0] > 0) {
            FastLED.addLeds<WS2812, PINS[0], RGB_ORDER>(fastLedPixels_.data(), pixelOffset,
                                                        lightsPerPin[0] * PIXELS_PER_LIGHT_);
            pixelOffset += lightsPerPin[0] * PIXELS_PER_LIGHT_;
        }
        if (lightsPerPin[1] > 0) {
            FastLED.addLeds<WS2812, PINS[1], RGB_ORDER>(fastLedPixels_.data(), pixelOffset,
                                                        lightsPerPin[1] * PIXELS_PER_LIGHT_);
            pixelOffset += lightsPerPin[1] * PIXELS_PER_LIGHT_;
        }
        if (lightsPerPin[2] > 0) {
            FastLED.addLeds<WS2812, PINS[2], RGB_ORDER>(fastLedPixels_.data(), pixelOffset,
                                                        lightsPerPin[2] * PIXELS_PER_LIGHT_);
            pixelOffset += lightsPerPin[2] * PIXELS_PER_LIGHT_;
        }
        if (lightsPerPin[3] > 0) {
            FastLED.addLeds<WS2812, PINS[3], RGB_ORDER>(fastLedPixels_.data(), pixelOffset,
                                                        lightsPerPin[3] * PIXELS_PER_LIGHT_);
            pixelOffset += lightsPerPin[3] * PIXELS_PER_LIGHT_;
        }

        FastLED.setCorrection(TypicalLEDStrip);
    }
};
