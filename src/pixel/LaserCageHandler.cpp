#include "LaserCageHandler.hpp"

static const char *TAG = "LaserCageHandler";

LaserCageHandler::LaserCageHandler(LedControl &ledControl, int laserCount)
    : laserCount_(laserCount), ledControl_(ledControl) {
    ledControl_.shutdown(0, false);
    ledControl.setIntensity(0, 15);
    ledControl_.clearDisplay(0);
}

void LaserCageHandler::write(const PixelFrame &frame) {
    assert(frame.size() == laserCount_);

    for (int i = 0; i < frame.size(); i++) {
        bool isOn = frame[i].r + frame[i].g + frame[i].b > 0;
        int row = i / 8;
        int column = i % 8;
        ledControl_.setLed(0, row, column, isOn);
    }
}

void LaserCageHandler::testLasers() {
    ESP_LOGI(TAG, "Testing Lasers...");
    ledControl_.clearDisplay(0);

    for (int i = 0; i < laserCount_; i++) {
        int row = i / 8;
        int column = i % 8;
        ledControl_.setLed(0, row, column, true);
        delay(100);
        ledControl_.clearDisplay(0);
    }
}