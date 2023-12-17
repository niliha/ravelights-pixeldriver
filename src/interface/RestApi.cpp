#include "RestApi.hpp"

#include <ArduinoJson.h>
#include <ESPmDNS.h>
#include <WiFi.h>

#include "config/PersistentStorage.hpp"

static const char *TAG = "RestApi";

RestApi::RestApi(int port) : server_(port), port_(port) {
    server_.on("/api/config", HTTP_GET, [this]() { this->on_get_config(); });
    server_.on("/api/config", HTTP_POST, [this]() { this->on_set_config(); });

    server_.enableDelay(false);
}

void RestApi::start() {
    server_.begin();
    ESP_LOGI(TAG, "REST API started on %s:%d", WiFi.localIP().toString().c_str(), port_);
    MDNS.addService("http", "tcp", port_);
}

void RestApi::handleReceived() {
    server_.handleClient();
}

void RestApi::on_get_config() {
    std::optional<OutputConfig> configOptional = PersistentStorage::loadOutputConfig();

    if (configOptional) {
        StaticJsonDocument<JSON_ARRAY_SIZE(4)> doc;
        for (const auto &output : *configOptional) {
            doc.add(output);
        }
        std::string configString;
        serializeJson(doc, configString);

        server_.send(200, "application/json", configString.c_str());
    } else {
        server_.send(502, "text/plain", "Output config not yet initialized");
    }
}

void RestApi::on_set_config() {
    std::string configString(server_.arg("plain").c_str());

    StaticJsonDocument<JSON_ARRAY_SIZE(4)> doc;
    auto error = deserializeJson(doc, configString);
    if (error) {
        ESP_LOGE(TAG, "Failed to parse JSON: %s", error.c_str());
        server_.send(400, "text/plain", "Failed to parse JSON");
        return;
    }

    if (!doc.is<JsonArray>()) {
        ESP_LOGE(TAG, "Invalid JSON format. Expected an array");
        server_.send(400, "text/plain", "Invalid JSON format. Expected an array");
        return;
    }

    if (doc.size() != OutputConfig::OUTPUT_COUNT) {
        ESP_LOGE(TAG, "Invalid array size %d", doc.size());
        server_.send(400, "text/plain", "Invalid array size");
        return;
    }

    server_.send(200, "text/plain", "Valid output configuration received. Applying...");

    OutputConfig newOutputConfig;
    for (int i = 0; i < newOutputConfig.size(); ++i) {
        newOutputConfig[i] = doc[i].as<uint32_t>();
    }

    std::optional<OutputConfig> currentOutputConfig = PersistentStorage::loadOutputConfig();
    if (currentOutputConfig && (*currentOutputConfig == newOutputConfig)) {
        ESP_LOGI(TAG,
                 "Not applying received pixels per output config since it is equal to current one (%d, %d, %d, %d)",
                 newOutputConfig[0], newOutputConfig[1], newOutputConfig[2], newOutputConfig[3]);
        server_.send(200, "text/plain",
                     "Not applying received pixels per output config since it is equal to current one");
        return;
    }

    if (!PersistentStorage::storeOutputConfig(newOutputConfig)) {
        server_.send(500, "text/plain", "Failed to store output config to flash");
        return;
    }

    server_.send(200, "text/plain", "Applying received output config...");
    ESP_LOGI(TAG, "Restarting ESP32 to apply new pixels per output config (%d, %d, %d, %d)...", newOutputConfig[0],
             newOutputConfig[1], newOutputConfig[2], newOutputConfig[3]);
    ESP.restart();
}
