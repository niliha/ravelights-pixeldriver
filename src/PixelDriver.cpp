#include "PixelDriver.hpp"

#include <esp32-hal.h>

static const char *TAG = "PixelDriver";

PixelDriver::PixelDriver(std::vector<std::shared_ptr<AbstractInterfaceHandler>> &interfaces,
                         BlockingRingBuffer<PixelFrame> &artnetQueue, AbstractPixelHandler &pixelHandler)
    : pixelHandler_(pixelHandler), interfaces_(interfaces), artnetQueue_(artnetQueue) {
    // The network task on core 1 does not yield to reduce latency.
    // Therefore, the watchdog on core 1 is not reset anymore, since the idle task is not resumed.
    // It is disabled to avoid watchdog timeouts resulting in a reboot.
    disableCore1WDT();
}

void PixelDriver::start() {
    // Start artnet task on core 1
    xTaskCreatePinnedToCore([](void *parameter) { static_cast<PixelDriver *>(parameter)->interfaceTask(); },
                            "interfaceTask", 4096, this, 1, NULL, 1);

    // Start the fastled task on core 0.
    xTaskCreatePinnedToCore([](void *parameter) { static_cast<PixelDriver *>(parameter)->pixelTask(); }, "pixelTask",
                            4096, this, 1, NULL, 0);
}

void PixelDriver::pixelTask() {
    ESP_LOGI(TAG, "pixelTask: started on core %d", xPortGetCoreID());

    unsigned long frameCount = 0;
    auto startMillis = millis();
    auto lastFrameMillis = millis();
    uint32_t minFramePeriodMillis = UINT32_MAX;
    uint32_t maxFramePeriodMillis = 0;

    while (true) {
        PixelFrame frame;
        artnetQueue_.pop(frame);
        pixelHandler_.write(frame);

        auto currentMillis = millis();
        auto framePeriodMillis = currentMillis - lastFrameMillis;
        lastFrameMillis = currentMillis;
        auto passedMillis = currentMillis - startMillis;

        if (framePeriodMillis < minFramePeriodMillis) {
            minFramePeriodMillis = framePeriodMillis;
        }

        if (framePeriodMillis > maxFramePeriodMillis) {
            maxFramePeriodMillis = framePeriodMillis;
        }

        frameCount++;

        if (passedMillis >= 5000) {
            ESP_LOGI(TAG, "Frames per second: %.2f; min, max frame period (ms): (%d, %d)",
                     (float)frameCount / (passedMillis / 1000.0), minFramePeriodMillis, maxFramePeriodMillis);

            frameCount = 0;
            minFramePeriodMillis = UINT32_MAX;
            maxFramePeriodMillis = 0;
            startMillis = currentMillis;
        }
    }
}

void PixelDriver::interfaceTask() {
    ESP_LOGI(TAG, "interfaceTask started on core %d", xPortGetCoreID());

    for (const auto &networkInterface : interfaces_) {
        networkInterface->start();
    }

    while (true) {
        for (const auto &networkInterface : interfaces_) {
            networkInterface->handleReceived();
        }
    }
}