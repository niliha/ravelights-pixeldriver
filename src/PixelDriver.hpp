#pragma once

#include <Preferences.h>
#include <variant>

#include "ArtnetHandler.hpp"
#include "FastLedHandler.hpp"
#include "PixelConfig.hpp"

template <const std::array<int, 4> &PINS, EOrder RGB_ORDER = RGB> class PixelDriver {
 public:
    PixelDriver(const std::array<uint8_t, 4> &defaultLightsPerPin, int pixelsPerLight = 144, int baudrate = 3000000,
                int frameQueueCapacity = 3)
        : fastLedHandler_(loadLightsPerOutputConfig(defaultLightsPerPin), pixelsPerLight),
          artnetQueue_(frameQueueCapacity), artnetHandler_(artnetQueue_, fastLedHandler_.getPixelCount(), baudrate),
          lastFrameMillis_(millis()) {
        // The Artnet task on core 0 does not block/sleep to reduce latency.
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
    static constexpr const char *LIGHTS_PER_OUTPUT_PREFERENCE_KEY = "output_config";

    FastLedHandler<PINS, RGB_ORDER> fastLedHandler_;
    ArtnetHandler artnetHandler_;
    BlockingRingBuffer<std::variant<PixelFrame, PixelConfig>> artnetQueue_;
    unsigned long lastFrameMillis_;

    std::array<uint8_t, 4> loadLightsPerOutputConfig(const std::array<uint8_t, 4> &defaultConfig) {
        Preferences preferences;
        preferences.begin(PREFERENCE_NAMESPACE, false);
        if (!preferences.isKey(LIGHTS_PER_OUTPUT_PREFERENCE_KEY)) {
            preferences.putBytes(LIGHTS_PER_OUTPUT_PREFERENCE_KEY, defaultConfig.data(), defaultConfig.size());
            preferences.end();
            Serial.printf("Using default lights per output config (%d, %d, %d, %d)\n", defaultConfig[0],
                          defaultConfig[1], defaultConfig[2], defaultConfig[3]);
            return defaultConfig;
        }

        std::array<uint8_t, 4> lightsPerOutput;
        preferences.getBytes(LIGHTS_PER_OUTPUT_PREFERENCE_KEY, lightsPerOutput.data(), lightsPerOutput.size());
        preferences.end();
        Serial.printf("Using lights per output config from flash (%d, %d, %d, %d)\n", lightsPerOutput[0],
                      lightsPerOutput[1], lightsPerOutput[2], lightsPerOutput[3]);
        return lightsPerOutput;
    }

    void applyPixelConfig(const PixelConfig &pixelConfig) {
        Preferences preferences;
        preferences.begin(PREFERENCE_NAMESPACE, false);
        if (preferences.isKey(LIGHTS_PER_OUTPUT_PREFERENCE_KEY)) {
            std::array<uint8_t, 4> oldLightsPerOutput;
            preferences.getBytes(LIGHTS_PER_OUTPUT_PREFERENCE_KEY, oldLightsPerOutput.data(),
                                 oldLightsPerOutput.size());
            if (pixelConfig.lightsPerOutput == oldLightsPerOutput) {
                Serial.printf("Not applying received output config since it is equal to current one (%d, %d, %d, %d)\n",
                              pixelConfig.lightsPerOutput[0], pixelConfig.lightsPerOutput[1],
                              pixelConfig.lightsPerOutput[2], pixelConfig.lightsPerOutput[3]);

                preferences.end();
                return;
            }
        }

        preferences.putBytes(LIGHTS_PER_OUTPUT_PREFERENCE_KEY, pixelConfig.lightsPerOutput.data(),
                             pixelConfig.lightsPerOutput.size());
        preferences.end();

        Serial.printf("Restarting ESP32 to apply received output config (%d, %d, %d, %d)...\n",
                      pixelConfig.lightsPerOutput[0], pixelConfig.lightsPerOutput[1], pixelConfig.lightsPerOutput[2],
                      pixelConfig.lightsPerOutput[3]);
        ESP.restart();
    }

    void fastledTask() {
        Serial.printf("fastledTask: started on core %d\n", xPortGetCoreID());

        while (true) {
            std::variant<PixelFrame, PixelConfig> pixelVariant;
            artnetQueue_.pop(pixelVariant);

            std::visit(
                [this](auto &&arg) {
                    using T = std::decay_t<decltype(arg)>;
                    if constexpr (std::is_same_v<T, PixelFrame>) {
                        fastLedHandler_.write(arg);
                        Serial.printf("%d ms since last frame\n", millis() - lastFrameMillis_);
                        lastFrameMillis_ = millis();
                    } else if constexpr (std::is_same_v<T, PixelConfig>) {
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