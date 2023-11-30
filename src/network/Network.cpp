#include "Arduino.h"
#include "WiFi.h"

static const char *TAG = "Network";

namespace Network {
bool connectToWifi(std::string ssid, std::string password) {
    ESP_LOGI(TAG, "Connecting to WiFi with SSID %s...", ssid.c_str());
    WiFi.begin(ssid.c_str(), password.c_str());

    // Wait 10 seconds for connection to be established
    for (unsigned i = 0; i < 5; i++) {
        if (WiFi.status() == WL_CONNECTED) {
            ESP_LOGI(TAG, "SUCCESS! IP address: %s", WiFi.localIP().toString().c_str());
            return true;
        }
        ESP_LOGI(TAG, "Still connecting to WiFi with SSID %s...", ssid.c_str());
        delay(2000);
    }

    ESP_LOGE(TAG, "Connection failed");
    return false;
}

bool initWifiAccessPoint(std::string ssid, std::string password) {
    ESP_LOGI(TAG, "Setting up access point with SSID %s...", ssid.c_str());
    WiFi.mode(WIFI_AP);
    if (WiFi.softAP(ssid.c_str(), password.c_str())) {
        // Default IP is 192.168.4.1
        ESP_LOGI(TAG, "SUCCESS! Access point IP address: %s", WiFi.softAPIP().toString().c_str());
        return true;
    }

    ESP_LOGE(TAG, "Setting up access point failed");
    return false;
}

}  // namespace Network
