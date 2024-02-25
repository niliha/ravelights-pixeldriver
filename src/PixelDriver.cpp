#include "PixelDriver.hpp"
#include "FpsLogger.hpp"

#include <esp32-hal.h>
#include <esp_log.h>
#include <esp_task.h>

static const char *TAG = "PixelDriver";

PixelDriver::PixelDriver(std::vector<std::shared_ptr<AbstractInterfaceHandler>> &interfaces,
                         BlockingRingBuffer<PixelFrame> &artnetQueue, AbstractPixelHandler &pixelHandler,
                         int pixelTaskCore, int pixelTaskPriority, int interfaceTaskCore, int interfaceTaskPriority)
    : INTERFACE_TASK_CORE_(interfaceTaskCore), INTERFACE_TASK_PRIORITY_(interfaceTaskPriority),
      PIXEL_TASK_CORE_(pixelTaskCore), PIXEL_TASK_PRIORITY_(pixelTaskPriority), pixelHandler_(pixelHandler),
      interfaces_(interfaces), artnetQueue_(artnetQueue) {
}

void PixelDriver::start() {
    // Run interface task on core 0.
    // Using a the same priority as the IDLE task such that it can yield to it and keep the watchdog fed
    xTaskCreatePinnedToCore([](void *parameter) { static_cast<PixelDriver *>(parameter)->interfaceTask(); },
                            "interfaceTask",
                            /* stack size */ 4096, this, /* priority */ tskIDLE_PRIORITY, NULL,
                            /* core */ INTERFACE_TASK_CORE_);

    // Run pixel task on core 1.
    // Using priority 19 or higher avoids being preempted by any built-in task
    xTaskCreatePinnedToCore([](void *parameter) { static_cast<PixelDriver *>(parameter)->pixelTask(); }, "pixelTask",
                            /* stack size */ 4096, this, /* priority */ 19, NULL, /* core */ PIXEL_TASK_CORE_);
}

void PixelDriver::pixelTask() {
    assert(xPortGetCoreID() == PIXEL_TASK_CORE_);

    ESP_LOGI(TAG, "pixelTask: started on core %d", PIXEL_TASK_CORE_);

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
    assert(xPortGetCoreID() == INTERFACE_TASK_CORE_);

    ESP_LOGI(TAG, "interfaceTask started on core %d", INTERFACE_TASK_CORE_);

    for (const auto &interface : interfaces_) {
        interface->start();
    }

    while (true) {
        for (const auto &interface : interfaces_) {
            interface->handleReceived();
        }
        taskYIELD();
    }
}
