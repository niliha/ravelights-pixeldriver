#pragma once

#include <FastLED.h>

#include "PixelFrame.hpp"
#include "pixel/AbstractPixelHandler.hpp"

#include <numeric>
#include <vector>

#ifndef CLOCKLESS_CHIPSET
#define CLOCKLESS_CHIPSET WS2812
#endif

template <const std::array<int, 4> &PINS, EOrder RGB_ORDER = RGB> class FastLedHandler : public AbstractPixelHandler {
 public:
    FastLedHandler(const OutputConfig &pixelsPerOutput)
        : PIXEL_COUNT_(std::accumulate(pixelsPerOutput.begin(), pixelsPerOutput.end(), 0)),
          fastLedPixels_(PIXEL_COUNT_) {
        int pixelOffset = 0;
        for (size_t i = 0; i < PINS.size(); i++) {
            if (pixelsPerOutput[i] > 0) {
                // Use a switch-case to handle each case at compile-time, since addLeds() template parameters, in
                // particular the pin number, must be known at compile-time.
                switch (i) {
                case 0:
                    FastLED.addLeds<CLOCKLESS_CHIPSET, PINS[0], RGB_ORDER>(fastLedPixels_.data(), pixelOffset,
                                                                           pixelsPerOutput[0]);
                    break;
                case 1:
                    FastLED.addLeds<CLOCKLESS_CHIPSET, PINS[1], RGB_ORDER>(fastLedPixels_.data(), pixelOffset,
                                                                           pixelsPerOutput[1]);
                    break;
                case 2:
                    FastLED.addLeds<CLOCKLESS_CHIPSET, PINS[2], RGB_ORDER>(fastLedPixels_.data(), pixelOffset,
                                                                           pixelsPerOutput[2]);
                    break;
                case 3:
                    FastLED.addLeds<CLOCKLESS_CHIPSET, PINS[3], RGB_ORDER>(fastLedPixels_.data(), pixelOffset,
                                                                           pixelsPerOutput[3]);
                    break;
                }
                pixelOffset += pixelsPerOutput[i];
            }
        }
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

    void testLights(int pixelsPerLight) {
        ESP_LOGI(TAG, "Testing Lights...");
        assert(PIXEL_COUNT_ % pixelsPerLight == 0);

        const std::vector<CRGB> colors{CRGB::DarkGray, CRGB::DarkRed, CRGB::DarkGreen, CRGB::DarkBlue};

        for (int i = 0; i < 3; i++) {
            int color_index = 0;
            for (int light_index = 0; light_index < PIXEL_COUNT_ / pixelsPerLight; light_index++) {
                for (int pixelOffset = 0; pixelOffset < pixelsPerLight; pixelOffset++) {
                    fastLedPixels_[light_index * pixelsPerLight + pixelOffset] = colors[color_index];
                }
                // Change color after each light
                color_index = (color_index + 1) % colors.size();
            }

            auto millisBefore = millis();
            FastLED.show();
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
    const int PIXEL_COUNT_;

    // The vector holding color values for each pixel.
    std::vector<CRGB> fastLedPixels_;
};