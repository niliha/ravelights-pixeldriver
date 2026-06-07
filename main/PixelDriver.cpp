#include "PixelDriver.hpp"
#include "FpsLogger.hpp"

#include <esp32-hal.h>
#include <esp_log.h>
#include <esp_task.h>

#include <algorithm>
#include <numeric>

static const char *TAG = "PixelDriver";

PixelDriver::PixelDriver(std::vector<std::shared_ptr<AbstractInterfaceHandler>> &interfaces,
                         BlockingRingBuffer<PixelFrame> &artnetQueue,
                         std::vector<std::shared_ptr<AbstractPixelHandler>> handlers,
                         int interfaceTaskCore, int interfaceTaskPriority, int pixelTaskCore, int pixelTaskPriority)
    : INTERFACE_TASK_CORE_(interfaceTaskCore), INTERFACE_TASK_PRIORITY_(interfaceTaskPriority),
      PIXEL_TASK_CORE_(pixelTaskCore), PIXEL_TASK_PRIORITY_(pixelTaskPriority), handlers_(std::move(handlers)),
      interfaces_(interfaces), artnetQueue_(artnetQueue) {
    TOTAL_PIXEL_COUNT_ = std::accumulate(handlers_.begin(), handlers_.end(), size_t{0},
                                         [](size_t sum, const auto &handler) { return sum + handler->pixelCount(); });
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
        assert(frame.size() == TOTAL_PIXEL_COUNT_);

        auto millisBeforeShow = millis();

        size_t offset = 0;
        for (const auto &handler : handlers_) {
            size_t count = handler->pixelCount();
            std::span<Pixel> region(frame.data() + offset, count);
            adjustBrightness(region, handler->maxBrightness());
            handler->write(region);
            offset += count;
        }

        fpsLogger.notifyFrameShown(millis() - millisBeforeShow);
    }
}

void PixelDriver::adjustBrightness(std::span<Pixel> pixels, uint8_t maxBrightness) {
    for (auto &pixel : pixels) {
        uint8_t maxChannel = std::max({pixel.r, pixel.g, pixel.b});

        if (maxChannel > maxBrightness) {
            float scale = static_cast<float>(maxBrightness) / static_cast<float>(maxChannel);

            pixel.r = static_cast<uint8_t>(pixel.r * scale);
            pixel.g = static_cast<uint8_t>(pixel.g * scale);
            pixel.b = static_cast<uint8_t>(pixel.b * scale);
        }
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
