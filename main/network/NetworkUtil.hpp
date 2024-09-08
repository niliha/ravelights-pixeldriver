#pragma once

#include <string>

namespace NetworkUtil {
bool connectToWifi(std::string ssid, std::string password);
bool initWifiAccessPoint(std::string ssid, std::string password);
}  // namespace NetworkUtil
