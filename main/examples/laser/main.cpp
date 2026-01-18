#include <ESPmDNS.h>

#include "PixelDriver.hpp"
#include "interface/artnet/ArtnetWifiHandler.hpp"
#include "network/NetworkUtil.hpp"
#include "network/WifiCredentials.hpp"
#include "pixel/LaserCageHandler.hpp"

static const char *TAG = "main";

// --- Config --------------------------------------------------------------------------------------
const int CHANNEL_COUNT = 4 * 16;

const int TLC5940_SIN_PIN = 17;
const int TLC5940_SCLK_PIN = 16;
const int TLC5940_XLAT_PIN = 4;
const int TLC5940_BLANK_PIN = 13;
const int TLC5940_GSCLK_PIN = 19;

// The instance ID used for mDNS discovery, must be without .local suffix
const std::string INSTANCE_ID = "pixeldriver-lasercage";

extern "C" void app_main() {
    initArduino();
    Serial.begin(115200);

    esp_log_level_set("wifi", ESP_LOG_WARN);
    esp_log_level_set("gpio", ESP_LOG_WARN);

    // --- Network ---------------------------------------------------------------------------------
    if (!NetworkUtil::connectToWifi(WifiCredentials::ssid, WifiCredentials::password)) {
        ESP_LOGE(TAG, "Rebooting because connecting to wifi with SSID %s failed", WifiCredentials::ssid.c_str());
        ESP.restart();
    }

    if (MDNS.begin(INSTANCE_ID.c_str())) {
        ESP_LOGI(TAG, "Started mDNS responder for instance id %s", INSTANCE_ID.c_str());
    } else {
        ESP_LOGE(TAG, "Rebooting because starting mDNS responder for instance id %s failed", INSTANCE_ID.c_str());
        ESP.restart();
    }

    // --- Interfaces ------------------------------------------------------------------------------
    BlockingRingBuffer<PixelFrame> artnetQueue(3);
    auto artnetWifi = std::make_shared<ArtnetWifiHandler>(artnetQueue, CHANNEL_COUNT);

    std::vector<std::shared_ptr<AbstractInterfaceHandler>> interfaces;
    interfaces.push_back(artnetWifi);

    // --- Pixel handler ---------------------------------------------------------------------------
    LaserCageHandler pixelHandler(CHANNEL_COUNT, TLC5940_SIN_PIN, TLC5940_SCLK_PIN, TLC5940_XLAT_PIN, TLC5940_BLANK_PIN,
                                  TLC5940_GSCLK_PIN);

    pixelHandler.testLasers();

    // --- Pixel driver ----------------------------------------------------------------------------
    PixelDriver pixelDriver(interfaces, artnetQueue, pixelHandler);
    pixelDriver.start();

    while (true) {
        // Stay in app_main() such that stack frame is not popped
        delay(1000);
    }
}
