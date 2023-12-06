#include "RestApi.hpp"

#include <ArduinoJson.h>
#include <WiFi.h>

#include "config/OutputConfgurator.hpp"

static const char *TAG = "RestApi";

RestApi::RestApi(int port) : server_(port), port_(port) {
    server_.on("/api/config", HTTP_GET, [this]() { this->on_get_config(); });
    server_.on("/api/config", HTTP_POST, [this]() { this->on_set_config(); });
}

void RestApi::start() {
    server_.begin();
    ESP_LOGI(TAG, "REST API started on %s:%d", WiFi.localIP().toString().c_str(), port_);
}

void RestApi::handleReceived() {
    server_.handleClient();
}

void RestApi::on_get_config() {
    std::optional<PixelOutputConfig> configOptional = OutputConfigurator::loadFromFlash();

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

    if (doc.size() != PixelOutputConfig::OUTPUT_CONFIG_SIZE) {
        ESP_LOGE(TAG, "Invalid array size %d", doc.size());
        server_.send(400, "text/plain", "Invalid array size");
        return;
    }

    PixelOutputConfig outputConfig;
    for (int i = 0; i < outputConfig.size(); ++i) {
        outputConfig[i] = doc[i].as<uint32_t>();
    }

    server_.send(200, "text/plain", "Valid output configuration received. Applying...");
    OutputConfigurator::applyToFlashAndReboot(outputConfig);
}
