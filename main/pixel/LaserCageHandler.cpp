#include "LaserCageHandler.hpp"

static const char *TAG = "LaserCageHandler";

LaserCageHandler::LaserCageHandler(int laserCount, int sin, int sclk, int xlat, int blank, int gsclk,
                                   uint8_t maxBrightness)
    : AbstractPixelHandler(maxBrightness), laserCount_(laserCount) {
    Tlc.init(0, sin, -1, sclk, xlat, blank, gsclk);
}

void LaserCageHandler::write(std::span<const Pixel> pixels) {
    assert(pixels.size() == static_cast<size_t>(laserCount_));

    Tlc.clear();
    for (size_t i = 0; i < pixels.size(); i++) {
        bool isOn = pixels[i].r + pixels[i].g + pixels[i].b > 0;
        if (isOn) {
            Tlc.set(i, 4095);
        }
    }

    Tlc.update();
}

void LaserCageHandler::testLasers() {
    ESP_LOGI(TAG, "Testing Lasers...");

    for (int i = 0; i < laserCount_; i++) {
        Tlc.clear();
        Tlc.set(i, 4095);
        Tlc.update();
        delay(250);
    }
}
