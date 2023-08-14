#pragma once

#include <Preferences.h>
#include <variant>

#include "config/OutputConfgurator.hpp"
#include "artnet/ArtnetHandler.hpp"
#include "artnet/PixelOutputConfig.hpp"
#include "fastled/FastLedHandler.hpp"

template <const std::array<int, 4> &PINS, EOrder RGB_ORDER = RGB> class PixelDriver {
 public:
    PixelDriver(const PixelOutputConfig &pixelsPerOutputFallback, int baudrate = 3000000, int frameQueueCapacity = 3)
        : fastLedHandler_(OutputConfigurator::load(pixelsPerOutputFallback)), artnetQueue_(frameQueueCapacity),
          artnetHandler_(artnetQueue_, fastLedHandler_.getPixelCount(), baudrate), lastFrameMillis_(millis()) {
        // The Artnet task on core 0 does not yield to reduce latency.
        // Therefore, the watchdog on core 0 is not reset anymore, since the idle task is not resumed.
        // It is disabled to avoid watchdog timeouts resulting in a reboot.
        disableCore0WDT();
    }

    void testLeds() {
        fastLedHandler_.testLeds();
    }

    void start() {
        // Start artnet task on core 0 (together with the WIFI service)
        xTaskCreatePinnedToCore([](void *parameter) { static_cast<PixelDriver *>(parameter)->artnetTask(); },
                                "artnetTask", 4096, this, 1, NULL, 0);

        // Start the fastled task on core 1.
        // Therefore it is not affected by WIFI interrupts from core 0 while writing to the LEDs
        // (avoiding flickering).
        xTaskCreatePinnedToCore([](void *parameter) { static_cast<PixelDriver *>(parameter)->fastledTask(); },
                                "fastledTask", 4096, this, 1, NULL, 1);
    }

 private:
    FastLedHandler<PINS, RGB_ORDER> fastLedHandler_;
    BlockingRingBuffer<std::variant<PixelFrame, PixelOutputConfig>> artnetQueue_;
    ArtnetHandler artnetHandler_;
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
                        OutputConfigurator::apply(arg);
                    }
                },
                pixelVariant);
        }
    }

    void artnetTask() {
        Serial.printf("artnetTask: started on core %d\n", xPortGetCoreID());

        while (true) {
            artnetHandler_.read();
        }
    }
};