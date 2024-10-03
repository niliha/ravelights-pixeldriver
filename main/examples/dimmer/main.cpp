#include <ESPmDNS.h>

#include "PixelDriver.hpp"
#include "interface/artnet/ArtnetWifiHandler.hpp"
#include "network/NetworkUtil.hpp"
#include "network/WifiCredentials.hpp"
#include "pixel/dimmer/AcDimmerHandler.hpp"
#include "pixel/dimmer/Mcp23s17TriacDriver.hpp"

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
const uint8_t MAX_BRIGHTNESS = 200;

// Custom channel mapping for the 9 * 7 lightbulbs light rings
// clang-format off
const std::vector<uint8_t> CUSTOM_CHANNEL_MAPPING = {
     8,  9, 10, 11, 12, 13, 14, // K1 (0 -> 6)
    15,  0,  1,  2,  3,  4,  5, // K2 (7 -> 13)
     6,  7, 32, 33, 34, 35, 36, // M1 (14 -> 20)
    37, 38, 39, 40, 41, 42, 43, // M2 (21 -> 27)
    44, 45, 46, 47, 48, 49, 50, // M2 (28 -> 34)
    51, 52, 53, 54, 55, 56, 57, // G1 (35 -> 41)
    58, 59, 60, 61, 62, 63, 24, // G2 (42 -> 48)
    25, 26, 27, 28, 29, 30, 31, // G3 (49 -> 55)
    16, 17, 18, 19, 20, 21, 22, // G4 (56 -> 62)
    23,                         // Unused
};
// clang-format on

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
    Mcp23s17TriacDriver triacDriver(CUSTOM_CHANNEL_MAPPING);
    AcDimmerHandler pixelHandler(CHANNEL_COUNT, ZERO_CROSSING_PIN, TRIAC_TASK_CORE, triacDriver, MAX_BRIGHTNESS);
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
