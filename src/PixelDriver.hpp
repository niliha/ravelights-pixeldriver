#pragma once

#include <Preferences.h>
#include <variant>

#include "artnet/ArtnetHandler.hpp"
#include "artnet/PixelOutputConfig.hpp"
#include "fastled/FastLedHandler.hpp"

template <const std::array<int, 4> &PINS, EOrder RGB_ORDER = RGB> class PixelDriver {
 public:
    PixelDriver(const PixelOutputConfig &pixelsPerOutputFallback, int baudrate = 3000000, int frameQueueCapacity = 3)
        : fastLedHandler_(loadOutputConfig(pixelsPerOutputFallback)), artnetQueue_(frameQueueCapacity),
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
    static constexpr const char *PREFERENCE_NAMESPACE = "ravelights";
    static constexpr const char *OUTPUT_CONFIG_PREFERENCE_KEY = "output_config";

    FastLedHandler<PINS, RGB_ORDER> fastLedHandler_;
    ArtnetHandler artnetHandler_;
    BlockingRingBuffer<std::variant<PixelFrame, PixelOutputConfig>> artnetQueue_;
    unsigned long lastFrameMillis_;
    PixelOutputConfig pixelsPerOutput_;

    PixelOutputConfig loadOutputConfig(const PixelOutputConfig &fallbackOutputConfig) {
        Preferences preferences;
        preferences.begin(PREFERENCE_NAMESPACE, false);
        if (!preferences.isKey(OUTPUT_CONFIG_PREFERENCE_KEY)) {
            preferences.putBytes(OUTPUT_CONFIG_PREFERENCE_KEY, fallbackOutputConfig.data(),
                                 fallbackOutputConfig.size());
            preferences.end();
            Serial.printf("Using default lights per output config (%d, %d, %d, %d)\n", fallbackOutputConfig[0],
                          fallbackOutputConfig[1], fallbackOutputConfig[2], fallbackOutputConfig[3]);
            pixelsPerOutput_ = fallbackOutputConfig;
            return fallbackOutputConfig;
        }

        preferences.getBytes(OUTPUT_CONFIG_PREFERENCE_KEY, pixelsPerOutput_.data(), pixelsPerOutput_.size());
        preferences.end();
        Serial.printf("Using lights per output config from flash (%d, %d, %d, %d)\n", pixelsPerOutput_[0],
                      pixelsPerOutput_[1], pixelsPerOutput_[2], pixelsPerOutput_[3]);
        return pixelsPerOutput_;
    }

    void applyPixelConfig(const PixelOutputConfig &outputConfig) {
        if (outputConfig == pixelsPerOutput_) {
            Serial.printf("Not applying received output config since it is equal to current one (%d, %d, %d, %d)\n",
                          outputConfig[0], outputConfig[1], outputConfig[2], outputConfig[3]);
            return;
        }

        Preferences preferences;
        preferences.begin(PREFERENCE_NAMESPACE, false);
        preferences.putBytes(OUTPUT_CONFIG_PREFERENCE_KEY, outputConfig.data(), outputConfig.size());
        preferences.end();

        Serial.printf("Restarting ESP32 to apply received output config (%d, %d, %d, %d)...\n", outputConfig[0],
                      outputConfig[1], outputConfig[2], outputConfig[3]);
        ESP.restart();
    }

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
                        Serial.printf("%d ms since last frame\n", millis() - lastFrameMillis_);
                        lastFrameMillis_ = millis();
                    } else if constexpr (std::is_same_v<T, PixelOutputConfig>) {
                        applyPixelConfig(arg);
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