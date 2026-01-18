#include "LaserCageHandler.hpp"

static const char *TAG = "LaserCageHandler";

LaserCageHandler::LaserCageHandler(int laserCount, int sin, int sclk, int xlat, int blank, int gsclk)
    : laserCount_(laserCount) {
    Tlc.init(0, sin, -1, sclk, xlat, blank, gsclk);
}

void LaserCageHandler::write(const PixelFrame &frame) {
    assert(frame.size() == laserCount_);

    Tlc.clear();
    for (int i = 0; i < frame.size(); i++) {
        bool isOn = frame[i].r + frame[i].g + frame[i].b > 0;
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
