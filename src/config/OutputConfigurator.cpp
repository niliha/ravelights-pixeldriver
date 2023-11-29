#include "OutputConfgurator.hpp"
#include "PreferencesRaii.hpp"

namespace OutputConfigurator {

static constexpr const char *PREFERENCE_NAMESPACE = "ravelights";
static constexpr const char *OUTPUT_CONFIG_PREFERENCE_KEY = "output_config";

PixelOutputConfig loadOrApplyFallback(const PixelOutputConfig &fallbackOutputConfig) {
    std::optional<PixelOutputConfig> outputConfigFromFlash = loadFromFlash();
    const auto &outputConfig = outputConfigFromFlash.value_or(fallbackOutputConfig);

    Serial.printf("Using pixels per output config from %s (%d, %d, %d, %d)\n",
                  outputConfigFromFlash ? "flash" : "fallback", outputConfig[0], outputConfig[1], outputConfig[2],
                  outputConfig[3]);

    if (!outputConfigFromFlash) {
        applyToFlash(fallbackOutputConfig);
    }

    return outputConfig;
}

std::optional<PixelOutputConfig> loadFromFlash() {
    PreferencesRaii preferences(PREFERENCE_NAMESPACE);
    PixelOutputConfig currentOutputConfig;

    if (preferences.isKey(OUTPUT_CONFIG_PREFERENCE_KEY)) {
        preferences.getBytes(OUTPUT_CONFIG_PREFERENCE_KEY, currentOutputConfig.data(),
                             currentOutputConfig.size() * sizeof(currentOutputConfig[0]));
        return currentOutputConfig;
    }

    return std::nullopt;
}

bool applyToFlash(const PixelOutputConfig &newOutputConfig) {
    std::optional<PixelOutputConfig> outputConfigFromFlash = loadFromFlash();

    if (outputConfigFromFlash && (*outputConfigFromFlash == newOutputConfig)) {
        return false;
    }

    PreferencesRaii preferences(PREFERENCE_NAMESPACE);
    preferences.putBytes(OUTPUT_CONFIG_PREFERENCE_KEY, newOutputConfig.data(),
                         newOutputConfig.size() * sizeof(newOutputConfig[0]));
    return true;
}

void applyToFlashAndReboot(const PixelOutputConfig &newOutputConfig) {
    bool wasApplied = applyToFlash(newOutputConfig);
    if (wasApplied) {
        Serial.printf("Restarting ESP32 to apply new pixels per output config (%d, %d, %d, %d)...\n",
                      newOutputConfig[0], newOutputConfig[1], newOutputConfig[2], newOutputConfig[3]);
        ESP.restart();
    } else {
        Serial.printf(
            "Not applying received pixels per output config since it is equal to current one (%d, %d, %d, %d)\n",
            newOutputConfig[0], newOutputConfig[1], newOutputConfig[2], newOutputConfig[3]);
    }
}
}  // namespace OutputConfigurator