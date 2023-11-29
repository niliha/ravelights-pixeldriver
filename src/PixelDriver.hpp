#pragma once

#include <memory>

#include "config/PixelOutputConfig.hpp"
#include "fastled/FastLedHandler.hpp"
#include "interface/AbstractInterfaceHandler.hpp"
#include "interface/artnet/BlockingRingBuffer.hpp"

template <const std::array<int, 4> &PINS, EOrder RGB_ORDER = RGB> class PixelDriver {
 public:
    PixelDriver(const PixelOutputConfig &pixelsPerOutput,
                std::vector<std::shared_ptr<AbstractInterfaceHandler>> &networkInterfaces,
                BlockingRingBuffer<PixelFrame> &artnetQueue)
        : fastLedHandler_(pixelsPerOutput), networkInterfaces_(networkInterfaces), artnetQueue_(artnetQueue),
          lastFrameMillis_(millis()) {
        // The network task on core 1 does not yield to reduce latency.
        // Therefore, the watchdog on core 1 is not reset anymore, since the idle task is not resumed.
        // It is disabled to avoid watchdog timeouts resulting in a reboot.
        disableCore1WDT();
    }

    void testPixels() {
        fastLedHandler_.testPixels();
    }

    void testLights(int pixelsPerLight) {
        fastLedHandler_.testLights(pixelsPerLight);
    }

    void start() {
        // Start artnet task on core 1
        xTaskCreatePinnedToCore([](void *parameter) { static_cast<PixelDriver *>(parameter)->networkTask(); },
                                "networkTask", 4096, this, 1, NULL, 1);

        // Start the fastled task on core 0.
        xTaskCreatePinnedToCore([](void *parameter) { static_cast<PixelDriver *>(parameter)->fastledTask(); },
                                "fastledTask", 4096, this, 1, NULL, 0);
    }

 private:
    FastLedHandler<PINS, RGB_ORDER> fastLedHandler_;
    std::vector<std::shared_ptr<AbstractInterfaceHandler>> &networkInterfaces_;
    BlockingRingBuffer<PixelFrame> &artnetQueue_;
    unsigned long lastFrameMillis_;

    void fastledTask() {
        Serial.printf("fastledTask: started on core %d\n", xPortGetCoreID());

        while (true) {
            PixelFrame frame;
            artnetQueue_.pop(frame);

            fastLedHandler_.write(frame);

            Serial.printf("%lu ms since last frame\n", millis() - lastFrameMillis_);
            lastFrameMillis_ = millis();
        }
    }

    void networkTask() {
        Serial.printf("networkTask started on core %d\n", xPortGetCoreID());

        for (const auto &networkInterface : networkInterfaces_) {
            networkInterface->start();
        }

        while (true) {
            for (const auto &networkInterface : networkInterfaces_) {
                networkInterface->handleReceived();
            }
        }
    }
};