#include "Arduino.h"
#include "WiFi.h"

namespace Network {
bool connectToWifi(std::string wifiSsid, std::string wifiPassword) {
    WiFi.begin(wifiSsid.c_str(), wifiPassword.c_str());
    Serial.println("Connecting to WiFi");
    // Wait for connection
    Serial.print("Connecting");
    for (unsigned i = 0; i <= 20; i++) {
        if (WiFi.status() == WL_CONNECTED) {
            Serial.print("SUCCESS! IP address:  ");
            Serial.println(WiFi.localIP());
            return true;
        }
        delay(500);
        Serial.print(".");
    }
    Serial.println("ERROR! Connection failed.");
    return false;
}

void initWifiAccessPoint(std::string wifiSsid, std::string wifiPassword) {
    Serial.println("Setting up access point...");
    WiFi.mode(WIFI_AP);  // Changing ESP32 wifi mode to AccessPoint
    WiFi.softAP(wifiSsid.c_str(), wifiPassword.c_str());
    IPAddress ipAddress = WiFi.softAPIP();
    Serial.print("AP IP address: ");
    Serial.println(ipAddress);  // Default IP is 192.168.4.1
}

}  // namespace Network
