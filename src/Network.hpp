#pragma once
#include <string>

namespace Network {
bool connectToWifi(std::string wifiSsid, std::string wifiPassword);
void initWifiAccessPoint(std::string wifiSsid, std::string wifiPassword);
}  // namespace Network
