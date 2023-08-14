#include "OutputConfgurator.hpp"
#include "PreferencesRaii.hpp"

namespace OutputConfigurator {

static constexpr const char *PREFERENCE_NAMESPACE = "ravelights";
static constexpr const char *OUTPUT_CONFIG_PREFERENCE_KEY = "output_config";

PixelOutputConfig pixelsPerOutput_;

PixelOutputConfig load(const PixelOutputConfig &fallbackOutputConfig) {
    PreferencesRaii preferences(PREFERENCE_NAMESPACE);

    if (preferences.isKey(OUTPUT_CONFIG_PREFERENCE_KEY)) {
        preferences.getBytes(OUTPUT_CONFIG_PREFERENCE_KEY, pixelsPerOutput_.data(), pixelsPerOutput_.size());
        Serial.printf("Using lights per output config from flash (%d, %d, %d, %d)\n", pixelsPerOutput_[0],
                      pixelsPerOutput_[1], pixelsPerOutput_[2], pixelsPerOutput_[3]);
    } else {
        preferences.putBytes(OUTPUT_CONFIG_PREFERENCE_KEY, fallbackOutputConfig.data(), fallbackOutputConfig.size());
        Serial.printf("Using default lights per output config (%d, %d, %d, %d)\n", fallbackOutputConfig[0],
                      fallbackOutputConfig[1], fallbackOutputConfig[2], fallbackOutputConfig[3]);
        pixelsPerOutput_ = fallbackOutputConfig;
    }

    return pixelsPerOutput_;
}

void apply(const PixelOutputConfig &outputConfig) {
    if (outputConfig == pixelsPerOutput_) {
        Serial.printf("Not applying received output config since it is equal to current one (%d, %d, %d, %d)\n",
                      outputConfig[0], outputConfig[1], outputConfig[2], outputConfig[3]);
        return;
    }

    PreferencesRaii preferences(PREFERENCE_NAMESPACE);
    preferences.putBytes(OUTPUT_CONFIG_PREFERENCE_KEY, outputConfig.data(), outputConfig.size());
    Serial.printf("Restarting ESP32 to apply received output config (%d, %d, %d, %d)...\n", outputConfig[0],
                  outputConfig[1], outputConfig[2], outputConfig[3]);
    ESP.restart();
}
}  // namespace OutputConfigurator