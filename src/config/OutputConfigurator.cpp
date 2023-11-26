#include "OutputConfgurator.hpp"
#include "PreferencesRaii.hpp"

namespace OutputConfigurator {

static constexpr const char *PREFERENCE_NAMESPACE = "ravelights";
static constexpr const char *OUTPUT_CONFIG_PREFERENCE_KEY = "output_config";

PixelOutputConfig loadOrApplyFallback(const PixelOutputConfig &fallbackOutputConfig) {
    std::optional<PixelOutputConfig> currentOutputConfig = load();
    if (currentOutputConfig.has_value()) {
        Serial.printf("Using pixels per output config from flash (%d, %d, %d, %d)\n", currentOutputConfig.value()[0],
                      currentOutputConfig.value()[1], currentOutputConfig.value()[2], currentOutputConfig.value()[3]);
        return currentOutputConfig.value();
    }

    Serial.printf("Using fallback pixels per output config (%d, %d, %d, %d)\n", fallbackOutputConfig[0],
                  fallbackOutputConfig[1], fallbackOutputConfig[2], fallbackOutputConfig[3]);
    apply(fallbackOutputConfig);
    return fallbackOutputConfig;
}

std::optional<PixelOutputConfig> load() {
    PreferencesRaii preferences(PREFERENCE_NAMESPACE);
    PixelOutputConfig currentOutputConfig;

    if (preferences.isKey(OUTPUT_CONFIG_PREFERENCE_KEY)) {
        preferences.getBytes(OUTPUT_CONFIG_PREFERENCE_KEY, currentOutputConfig.data(),
                             currentOutputConfig.size() * sizeof(currentOutputConfig[0]));
        return currentOutputConfig;
    }

    return std::nullopt;
}

void apply(const PixelOutputConfig &newOutputConfig) {
    std::optional<PixelOutputConfig> currentOutputConfig = load();
    if (currentOutputConfig.has_value() && currentOutputConfig.value() == newOutputConfig) {
        Serial.printf(
            "Not applying received pixels per output config since it is equal to current one (%d, %d, %d, %d)\n",
            newOutputConfig[0], newOutputConfig[1], newOutputConfig[2], newOutputConfig[3]);
        return;
    }

    {
        PreferencesRaii preferences(PREFERENCE_NAMESPACE);
        preferences.putBytes(OUTPUT_CONFIG_PREFERENCE_KEY, newOutputConfig.data(),
                             newOutputConfig.size() * sizeof(newOutputConfig[0]));
    }
}

void applyAndReboot(const PixelOutputConfig &newOutputConfig) {
    apply(newOutputConfig);
    Serial.printf("Restarting ESP32 to apply new pixels per output config (%d, %d, %d, %d)...\n", newOutputConfig[0],
                  newOutputConfig[1], newOutputConfig[2], newOutputConfig[3]);
    ESP.restart();
}
}  // namespace OutputConfigurator