#include "LaserCageHandler.hpp"

static const char *TAG = "LaserCageHandler";

LaserCageHandler::LaserCageHandler(int laserCount) : laserCount_(laserCount) {
    // Tlc.init(0);

    Tlc.init(0, 17, 25, 16, 4, 13, 19);
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
    for (int i = 0; i < laserCount_; i++) {
        Tlc.clear();
        Tlc.set(i, 4095);
        Tlc.update();
        delay(500);
    }
    /*
    ESP_LOGI(TAG, "Testing Lasers...");
    ledControl_.clearDisplay(0);

    for (int i = 0; i < laserCount_; i++) {
        int row = i / 8;
        int column = i % 8;
        ledControl_.setLed(0, row, column, true);
        delay(100);
        ledControl_.clearDisplay(0);
    }
    */
}