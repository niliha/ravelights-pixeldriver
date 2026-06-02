#include <ESPmDNS.h>

#include "PixelDriver.hpp"
#include "config/PersistentStorage.hpp"
#include "interface/artnet/ArtnetWifiHandler.hpp"
#include "network/NetworkUtil.hpp"
#include "network/WifiCredentials.hpp"
#include "pixel/dc_dimmer/DcDimmerHandler.hpp"
#include <Adafruit_TLC59711.h>

static const char *TAG = "main";

// --- Config --------------------------------------------------------------------------------------
const int CHANNEL_COUNT = 12;

// The instance ID used for mDNS discovery, must be without .local suffix
const std::string instanceIdFallback = "pixeldriver-dc-dimmer-1";

const int DRIVER_COUNT = 1;
const int SCLK_PIN = 18;
const int MOSI_PIN = 23;

// PWM Value for TLC69711 can be between 0 and UINT16_MAX.
// The lower and upper bounds should be set according to the used light bulbs.
const uint16_t MIN_PWM_VALUE = 0;
const uint16_t MAX_PWM_VALUE = UINT16_MAX / 2;

extern "C" void app_main() {
    initArduino();
    Serial.begin(115200);

    esp_log_level_set("wifi", ESP_LOG_WARN);
    esp_log_level_set("gpio", ESP_LOG_WARN);

    // --- Persistent storage ----------------------------------------------------------------------
    // PersistentStorage::clear();
    auto instanceId = PersistentStorage::loadOrStoreFallbackInstanceId(instanceIdFallback);

    // --- Network ---------------------------------------------------------------------------------
    if (!NetworkUtil::connectToWifi(WifiCredentials::ssid, WifiCredentials::password)) {
        ESP_LOGE(TAG, "Rebooting because connecting to wifi with SSID %s failed", WifiCredentials::ssid.c_str());
        ESP.restart();
    }

    if (MDNS.begin(instanceId.c_str())) {
        ESP_LOGI(TAG, "Started mDNS responder for instance id %s", instanceId.c_str());
    } else {
        ESP_LOGE(TAG, "Rebooting because starting mDNS responder for instance id %s failed", instanceId.c_str());
        ESP.restart();
    }

    // --- Interfaces ------------------------------------------------------------------------------
    BlockingRingBuffer<PixelFrame> artnetQueue(1);
    auto artnetWifi = std::make_shared<ArtnetWifiHandler>(artnetQueue, CHANNEL_COUNT);

    std::vector<std::shared_ptr<AbstractInterfaceHandler>> interfaces;
    interfaces.push_back(artnetWifi);

    // --- Pixel handler ---------------------------------------------------------------------------
    // TODO: Use proper hardware SPI instead of bit banging
    Adafruit_TLC59711 tlc59711(DRIVER_COUNT, SCLK_PIN, MOSI_PIN);
    DcDimmerHandler pixelHandler(tlc59711, CHANNEL_COUNT, MIN_PWM_VALUE, MAX_PWM_VALUE);
    pixelHandler.testLights();

    // --- Pixel driver ----------------------------------------------------------------------------
    PixelDriver pixelDriver(interfaces, artnetQueue, pixelHandler);
    pixelDriver.start();

    while (true) {
        // Stay in app_main() such that stack frame is not popped
        delay(1000);
    }
}
