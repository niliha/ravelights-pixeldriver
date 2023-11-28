#pragma once

#include <Preferences.h>
#include <variant>

#include "artnet/ArtnetHandler.hpp"
#include "artnet/PixelOutputConfig.hpp"
#include "config/OutputConfgurator.hpp"
#include "fastled/FastLedHandler.hpp"
#include "interface/AbstractNetworkInterface.hpp"

template <const std::array<int, 4> &PINS, EOrder RGB_ORDER = RGB> class PixelDriver {
 public:
    PixelDriver(const PixelOutputConfig &pixelsPerOutput,
                std::vector<std::unique_ptr<AbstractNetworkInterface>> networkInterfaces,
                BlockingRingBuffer<std::variant<PixelFrame, PixelOutputConfig>> &artnetQueue)
        : fastLedHandler_(pixelsPerOutput), networkInterfaces_(std::move(networkInterfaces)), artnetQueue_(artnetQueue),
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
    std::vector<std::unique_ptr<AbstractNetworkInterface>> networkInterfaces_;
    BlockingRingBuffer<std::variant<PixelFrame, PixelOutputConfig>> &artnetQueue_;
    unsigned long lastFrameMillis_;

    void fastledTask() {
        Serial.printf("fastledTask: started on core %d\n", xPortGetCoreID());

        while (true) {
            std::variant<PixelFrame, PixelOutputConfig> pixelVariant;
            artnetQueue_.pop(pixelVariant);

            std::visit(
                [this](auto &&arg) {
                    using T = std::decay_t<decltype(arg)>;
                    if constexpr (std::is_same_v<T, PixelFrame>) {
                        fastLedHandler_.write(arg);
                        Serial.printf("%lu ms since last frame\n", millis() - lastFrameMillis_);
                        lastFrameMillis_ = millis();
                    } else if constexpr (std::is_same_v<T, PixelOutputConfig>) {
                        OutputConfigurator::applyToFlashAndReboot(arg);
                    }
                },
                pixelVariant);
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