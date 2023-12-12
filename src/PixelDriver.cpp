#include "PixelDriver.hpp"
#include "FpsLogger.hpp"

#include <esp32-hal.h>
#include <esp_task.h>

static const char *TAG = "PixelDriver";

PixelDriver::PixelDriver(std::vector<std::shared_ptr<AbstractInterfaceHandler>> &interfaces,
                         BlockingRingBuffer<PixelFrame> &artnetQueue, AbstractPixelHandler &pixelHandler)
    : pixelHandler_(pixelHandler), interfaces_(interfaces), artnetQueue_(artnetQueue) {
}

void PixelDriver::start() {
    // Run interface task on core 0.
    // Using a lower priority than the lwIP TCP/IP task since most interface handlers rely on the network stack.
    xTaskCreatePinnedToCore([](void *parameter) { static_cast<PixelDriver *>(parameter)->interfaceTask(); }, "interfaceTask",
     /* stack size */ 4096, this, /* priority */ ESP_TASK_TCPIP_PRIO - 1, NULL,
                            /* core */ 0);

    // Run pixel task on core 1.
    // Using priority 19 or higher avoids being preempted by any built-in task
    xTaskCreatePinnedToCore([](void *parameter) { static_cast<PixelDriver *>(parameter)->pixelTask(); }, "pixelTask",
                            /* stack size */ 4096, this, /* priority */ 19, NULL, /* core */ 1);
}

void PixelDriver::pixelTask() {
    ESP_LOGI(TAG, "pixelTask: started on core %d", xPortGetCoreID());

    FpsLogger fpsLogger;

    while (true) {
        PixelFrame frame;
        artnetQueue_.pop(frame);

        auto millisBeforeShow = millis();
        pixelHandler_.write(frame);
        fpsLogger.notifyFrameShown(millis() - millisBeforeShow);
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