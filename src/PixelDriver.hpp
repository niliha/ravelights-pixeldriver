#pragma once

#include <FastLED.h>

template <int PIN_COUNT, const std::array<int, PIN_COUNT> &PINS, EOrder RGB_ORDER = RGB>
class PixelDriver {
 public:
    PixelDriver(const std::array<int, PIN_COUNT> &lightsPerPin, int pixelsPerLight = 144,
                uint8_t maxBrightness = 255, bool debug = false)
        : lightsPerPin_(lightsPerPin), MAX_BRIGHTNESS_(maxBrightness), DEBUG_(debug),
          PIXELS_PER_LIGHT_(pixelsPerLight),
          pixels_(pixelsPerLight *
                  std::accumulate(lightsPerPin.begin(), lightsPerPin.end(), 0, std::plus<int>())) {
        setupFastled();
        while (true) {
            testLeds();
        }
    }

    void testLeds() {
        Serial.println("Testing LEDs...");
        std::vector<CRGB> colors{CRGB::Red, CRGB::Green, CRGB::Blue};
        for (const auto color : colors) {
            auto timeBefore = millis();
            FastLED.showColor(color);
            auto passedTime = millis() - timeBefore;
            Serial.print("show() took ");
            Serial.print(passedTime);
            Serial.println(" ms");
            delay(500);
            FastLED.clear(true);
            delay(500);
        }
        delay(2000);
    }

 private:
    const int PIXELS_PER_LIGHT_;
    std::array<int, PIN_COUNT> lightsPerPin_;
    const uint8_t MAX_BRIGHTNESS_;
    const bool DEBUG_;
    std::vector<CRGB> pixels_;

    void setupFastled() {
        static_assert(PIN_COUNT == 4 && "setupFastLed() is hardcoded to handle exactly 4 pins!");
        // We can't use a loop here since addLeds() template parameters must be known at
        // compile-time
        int pixelOffset = 0;
        if (lightsPerPin_[0] > 0) {
            FastLED.addLeds<WS2812, PINS[0], RGB_ORDER>(pixels_.data(), pixelOffset,
                                                        lightsPerPin_[0] * PIXELS_PER_LIGHT_);
            pixelOffset += lightsPerPin_[0] * PIXELS_PER_LIGHT_;
        }
        if (lightsPerPin_[1] > 0) {
            FastLED.addLeds<WS2812, PINS[1], RGB_ORDER>(pixels_.data(), pixelOffset,
                                                        lightsPerPin_[1] * PIXELS_PER_LIGHT_);
            pixelOffset += lightsPerPin_[1] * PIXELS_PER_LIGHT_;
        }
        if (lightsPerPin_[2] > 0) {
            FastLED.addLeds<WS2812, PINS[2], RGB_ORDER>(pixels_.data(), pixelOffset,
                                                        lightsPerPin_[2] * PIXELS_PER_LIGHT_);
            pixelOffset += lightsPerPin_[2] * PIXELS_PER_LIGHT_;
        }
        if (lightsPerPin_[3] > 0) {
            FastLED.addLeds<WS2812, PINS[3], RGB_ORDER>(pixels_.data(), pixelOffset,
                                                        lightsPerPin_[3] * PIXELS_PER_LIGHT_);
            pixelOffset += lightsPerPin_[3] * PIXELS_PER_LIGHT_;
        }
        // Set maximum brightness (0 - 255)
        FastLED.setBrightness(MAX_BRIGHTNESS_);
        // FastLED.setMaxRefreshRate(50);
    }
};
