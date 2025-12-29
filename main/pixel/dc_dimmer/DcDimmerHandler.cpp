#include "DcDimmerHandler.hpp"

#include <numeric>

static const char *TAG = "DcDimmerHandler";

DcDimmerHandler::DcDimmerHandler(Adafruit_TLC59711 &tlc59711, const int lightCount, const uint8_t maxBrightness)
    : LIGHT_COUNT_(lightCount), MAX_BRIGHTNESS_(maxBrightness), tlc59711_(tlc59711) {
    tlc59711_.begin();
}

void DcDimmerHandler::write(const PixelFrame &frame) {
    assert(frame.size() == LIGHT_COUNT_);

    for (int i = 0; i < frame.size(); i++) {
        auto pixel = frame[i];
        uint8_t brightness8Bit = std::min(std::max({pixel.r, pixel.g, pixel.b}), MAX_BRIGHTNESS_);
        uint16_t brightness16Bit = map(brightness8Bit, 0, UINT8_MAX, 0, UINT16_MAX);

        tlc59711_.setPWM(i, brightness16Bit);
    }

    tlc59711_.write();
}

void DcDimmerHandler::testLights() {
    ESP_LOGI(TAG, "Testing lights...");

    uint16_t maxBrightness16Bit = map(MAX_BRIGHTNESS_, 0, UINT8_MAX, 0, UINT16_MAX);

    for (int brightness = 0; brightness < maxBrightness16Bit; brightness += 255 / 2) {
        for (int channel = 0; channel < LIGHT_COUNT_; channel++) {
            tlc59711_.setPWM(channel, brightness);
        }

        tlc59711_.write();
        delay(10);
    }
    delay(500);

    for (int brightness = maxBrightness16Bit; brightness > 0; brightness -= 255 / 2) {
        for (int channel = 0; channel < LIGHT_COUNT_; channel++) {
            tlc59711_.setPWM(channel, brightness);
        }

        tlc59711_.write();
        delay(10);
    }
    delay(500);
}
