#include <ESPmDNS.h>

#include "PixelDriver.hpp"
#include "interface/artnet/ArtnetWifiHandler.hpp"
#include "network/NetworkUtil.hpp"
#include "network/WifiCredentials.hpp"
#include "pixel/dimmer/AcDimmerHandler.hpp"

static const char *TAG = "main";

// --- Config --------------------------------------------------------------------------------------
const int CHANNEL_COUNT = 9 * 7;
const int ZERO_CROSSING_PIN = 4;
const int TRIAC_TASK_CORE = 1;

const int INTERFACE_TASK_CORE = 0;
const int INTERFACE_TASK_PRIORITY = tskIDLE_PRIORITY;
const int PIXEL_TASK_CORE = 0;
const int PIXEL_TASK_PRIORITY = tskIDLE_PRIORITY;

// The instance ID used for mDNS discovery, must be without .local suffix
const std::string INSTANCE_ID = "pixeldriver-dimmer";

// The brightness scaling factor. 0 = minimum brightness, 255 = maximum brightness
const uint8_t BRIGHTNESS = 127;

extern "C" void app_main() {
    initArduino();
    Serial.begin(115200);

    esp_log_level_set("wifi", ESP_LOG_WARN);

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
    AcDimmerHandler pixelHandler(CHANNEL_COUNT, ZERO_CROSSING_PIN, TRIAC_TASK_CORE, BRIGHTNESS);
    pixelHandler.testLights();

    // --- Pixel driver ----------------------------------------------------------------------------
    PixelDriver pixelDriver(interfaces, artnetQueue, pixelHandler, INTERFACE_TASK_CORE, INTERFACE_TASK_PRIORITY,
                            PIXEL_TASK_CORE, PIXEL_TASK_PRIORITY);
    pixelDriver.start();

    while (true) {
        // Stay in app_main() such that stack frame is not popped
        delay(1000);
    }
}
