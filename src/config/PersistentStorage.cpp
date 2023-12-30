#include "PersistentStorage.hpp"

#include <nvs_flash.h>

#include "PreferencesRaii.hpp"

static const char *TAG = "PersistentStorage";

namespace PersistentStorage {

static constexpr const char *PREFERENCE_NAMESPACE = "ravelights";
static constexpr const char *OUTPUT_CONFIG_PREFERENCE_KEY = "output_config";
static constexpr const char *INSTANCE_ID_PREFERENCE_KEY = "hostname";

OutputConfig loadOrStoreFallbackOutputConfig(const OutputConfig &fallbackOutputConfig) {
    std::optional<OutputConfig> outputConfigFromFlash = loadOutputConfig();
    const auto &outputConfig = outputConfigFromFlash.value_or(fallbackOutputConfig);

    ESP_LOGI(TAG, "Using pixels per output config from %s (%d, %d, %d, %d)",
             outputConfigFromFlash ? "flash" : "fallback", outputConfig[0], outputConfig[1], outputConfig[2],
             outputConfig[3]);

    if (!outputConfigFromFlash) {
        storeOutputConfig(fallbackOutputConfig);
    }

    return outputConfig;
}

std::optional<OutputConfig> loadOutputConfig() {
    PreferencesRaii preferences(PREFERENCE_NAMESPACE);
    OutputConfig currentOutputConfig;

    if (!preferences.isKey(OUTPUT_CONFIG_PREFERENCE_KEY)) {
        return std::nullopt;
    }

    int readBytesCount =
        preferences.getBytes(OUTPUT_CONFIG_PREFERENCE_KEY, currentOutputConfig.data(), OutputConfig::FLASH_SIZE);
    if (readBytesCount != OutputConfig::FLASH_SIZE) {
        ESP_LOGW(TAG, "Failed to read output config from flash. Read %d bytes, expected %d", readBytesCount,
                 OutputConfig::FLASH_SIZE);
        return std::nullopt;
    }

    return currentOutputConfig;
}

bool storeOutputConfig(const OutputConfig &newOutputConfig) {
    PreferencesRaii preferences(PREFERENCE_NAMESPACE);
    if (!preferences.putBytes(OUTPUT_CONFIG_PREFERENCE_KEY, newOutputConfig.data(), OutputConfig::FLASH_SIZE)) {
        ESP_LOGE(TAG, "Failed to write output config to flash");
        return false;
    }

    ESP_LOGI(TAG, "Wrote pixels per output config to flash (%d, %d, %d, %d)", newOutputConfig[0], newOutputConfig[1],
             newOutputConfig[2], newOutputConfig[3]);
    return true;
}

std::string loadOrStoreFallbackInstanceId(const std::string fallbackInstanceId) {
    std::optional<std::string> currentInstanceId = loadInstanceId();
    const auto &instanceId = currentInstanceId.value_or(fallbackInstanceId);

    ESP_LOGI(TAG, "Using instance id from %s: %s", currentInstanceId ? "flash" : "fallback", instanceId.c_str());

    if (!currentInstanceId) {
        storeInstanceId(fallbackInstanceId);
    }

    return instanceId;
}

std::optional<std::string> loadInstanceId() {
    PreferencesRaii preferences(PREFERENCE_NAMESPACE);

    if (!preferences.isKey(INSTANCE_ID_PREFERENCE_KEY)) {
        return std::nullopt;
    }

    String instanceId = preferences.getString(INSTANCE_ID_PREFERENCE_KEY);
    if (instanceId.isEmpty()) {
        ESP_LOGE(TAG, "Failed to read instance id from flash");
        return std::nullopt;
    }

    return std::string(instanceId.c_str());
}

bool storeInstanceId(const std::string &newInstanceId) {
    PreferencesRaii preferences(PREFERENCE_NAMESPACE);

    if (!preferences.putString(INSTANCE_ID_PREFERENCE_KEY, newInstanceId.c_str())) {
        ESP_LOGE(TAG, "Failed to write instance id %s to flash", newInstanceId.c_str());
        return false;
    }

    ESP_LOGI(TAG, "Wrote instance id %s to flash", newInstanceId.c_str());
    return true;
}

void clear() {
    nvs_flash_erase();
    nvs_flash_init();
}
}  // namespace PersistentStorage