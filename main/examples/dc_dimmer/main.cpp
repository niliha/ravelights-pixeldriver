#include <ESPmDNS.h>

#include "PixelDriver.hpp"
#include "interface/artnet/ArtnetWifiHandler.hpp"
#include "network/NetworkUtil.hpp"
#include "network/WifiCredentials.hpp"
#include "pixel/dc_dimmer/DcDimmerHandler.hpp"
#include <Adafruit_TLC59711.h>

static const char *TAG = "main";

// --- Config --------------------------------------------------------------------------------------
const int CHANNEL_COUNT = 12;

// The instance ID used for mDNS discovery, must be without .local suffix
const std::string INSTANCE_ID = "pixeldriver-dc-dimmer-1";

// 0 = minimum brightness, 255 = maximum brightness
const uint8_t MAX_BRIGHTNESS = 200;

const int DRIVER_COUNT = 1;
const int SCLK_PIN = 18;
const int MOSI_PIN = 23;

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
    // TODO: Use proper hardware SPI instead of bit banging
    Adafruit_TLC59711 tlc59711(DRIVER_COUNT, SCLK_PIN, MOSI_PIN);
    DcDimmerHandler pixelHandler(tlc59711, CHANNEL_COUNT);
    pixelHandler.testLights();

    // --- Pixel driver ----------------------------------------------------------------------------
    PixelDriver pixelDriver(interfaces, artnetQueue, pixelHandler, MAX_BRIGHTNESS);
    pixelDriver.start();

    while (true) {
        // Stay in app_main() such that stack frame is not popped
        delay(1000);
    }
}
