#include "DcDimmerHandler.hpp"

#include <numeric>

static const char *TAG = "DcDimmerHandler";

DcDimmerHandler::DcDimmerHandler(Adafruit_TLC59711 &tlc59711, const int lightCount, const uint16_t minPwmValue,
                                 const uint16_t maxPwmValue)
    : lightCount_(lightCount), minPwmValue_(minPwmValue), maxPwmValue_(maxPwmValue), tlc59711_(tlc59711) {
    tlc59711_.begin();
}

void DcDimmerHandler::write(const PixelFrame &frame) {
    assert(frame.size() == lightCount_);

    for (int i = 0; i < frame.size(); i++) {
        auto pixel = frame[i];
        uint8_t brightness = std::max({pixel.r, pixel.g, pixel.b});
        uint16_t pwmValue = map(brightness, 0, UINT8_MAX, minPwmValue_, maxPwmValue_);

        tlc59711_.setPWM(i, pwmValue);
    }

    tlc59711_.write();
}

void DcDimmerHandler::testLights() {
    ESP_LOGI(TAG, "Testing lights...");

    uint16_t maxBrightness16Bit = UINT16_MAX / 4;

    for (int brightness = 0; brightness < maxBrightness16Bit; brightness += 255 / 2) {
        for (int channel = 0; channel < lightCount_; channel++) {
            tlc59711_.setPWM(channel, brightness);
        }

        tlc59711_.write();
        delay(10);
    }
    delay(500);

    for (int brightness = maxBrightness16Bit; brightness > 0; brightness -= 255 / 2) {
        for (int channel = 0; channel < lightCount_; channel++) {
            tlc59711_.setPWM(channel, brightness);
        }

        tlc59711_.write();
        delay(10);
    }
    delay(500);
}
