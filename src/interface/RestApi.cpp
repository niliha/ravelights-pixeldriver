#include <ArduinoJson.h>
#include <WiFi.h>

#include "RestApi.hpp"
#include "config/OutputConfgurator.hpp"

RestApi::RestApi(int port) : server_(port), port_(port) {
    server_.on("/api/config", HTTP_GET, [this]() { this->on_get_config(); });
    server_.on("/api/config", HTTP_POST, [this]() { this->on_set_config(); });
}

void RestApi::start() {
    server_.begin();
    Serial.printf("REST API started on %s:%d\n", WiFi.localIP().toString().c_str(), port_);
}

void RestApi::handleReceived() {
    server_.handleClient();
}

void RestApi::on_get_config() {
    std::optional<PixelOutputConfig> configOptional = OutputConfigurator::loadFromFlash();

    if (configOptional) {
        StaticJsonDocument<JSON_ARRAY_SIZE(4)> doc;
        for (const auto& output : *configOptional) {
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
        Serial.printf("Failed to parse JSON: %s\n", error.c_str());
        server_.send(400, "text/plain", "Failed to parse JSON");
        return;
    }

    if (!doc.is<JsonArray>()) {
        Serial.println("Invalid JSON format. Expected an array");
        server_.send(400, "text/plain", "Invalid JSON format. Expected an array");
        return;
    }

    if (doc.size() != PIXEL_OUTPUT_CONFIG_SIZE) {
        Serial.printf("Invalid array size %d\n", doc.size());
        server_.send(400, "text/plain", "Invalid array size. Must be 4");
        return;
    }

    PixelOutputConfig outputConfig;
    for (int i = 0; i < outputConfig.size(); ++i ) {
        outputConfig[i] =doc[i].as<uint32_t>();
    }

    OutputConfigurator::applyToFlashAndReboot(outputConfig);
}
