#pragma once
#include <string>

namespace Network {
bool connectToWifi(std::string ssid, std::string password);
bool initWifiAccessPoint(std::string ssid, std::string password);
}  // namespace Network
