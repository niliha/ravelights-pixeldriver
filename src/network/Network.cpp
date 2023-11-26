#include "Arduino.h"
#include "WiFi.h"

namespace Network {
bool connectToWifi(std::string ssid, std::string password) {
    Serial.printf("Connecting to WiFi with SSID %s...\n", ssid.c_str());
    WiFi.begin(ssid.c_str(), password.c_str());

    // Wait 10 seconds for connection to be established
    for (unsigned i = 0; i <= 20; i++) {
        if (WiFi.status() == WL_CONNECTED) {
            Serial.printf("\nSUCCESS! IP address: ");
            Serial.println(WiFi.localIP());
            return true;
        }
        delay(500);
        Serial.print(".");
    }

    Serial.printf("\nERROR: Connection failed.\n");
    return false;
}

void initWifiAccessPoint(std::string ssid, std::string password) {
    Serial.printf("Setting up access point with SSID %s...\n", ssid.c_str());
    WiFi.mode(WIFI_AP);
    WiFi.softAP(ssid.c_str(), password.c_str());

    // Default IP is 192.168.4.1
    Serial.print("AP IP address: ");
    Serial.println(WiFi.softAPIP());
}

}  // namespace Network
