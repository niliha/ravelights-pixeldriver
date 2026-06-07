#include <ESPmDNS.h>

#include "PixelDriver.hpp"
#include "config/PersistentStorage.hpp"
#include "interface/artnet/ArtnetWifiHandler.hpp"
#include "network/NetworkUtil.hpp"
#include "network/WifiCredentials.hpp"
#include "pixel/SinglePinFastLedHandler.hpp"
#include "pixel/dc_dimmer/DcDimmerHandler.hpp"
#include <Adafruit_TLC59711.h>

static const char *TAG = "main";

// --- LED strip config ----------------------------------------------------------------------------
constexpr int LED_PIN = 19;
const int LED_PIXEL_COUNT = 5 * 60 + 2 * 144;
const EOrder RGB_ORDER = EOrder::RGB;

// 0 = minimum brightness, 255 = maximum brightness
const uint8_t LED_MAX_BRIGHTNESS = 127;

// --- DC dimmer config ----------------------------------------------------------------------------
const int DIMMER_CHANNEL_COUNT = 12;
const int DIMMER_DRIVER_COUNT = 1;
const int DIMMER_SCLK_PIN = 18;
const int DIMMER_MOSI_PIN = 23;

const uint16_t DIMMER_MIN_PWM = 0;
const uint16_t DIMMER_MAX_PWM = UINT16_MAX / 2;

// 0 = minimum brightness, 255 = maximum brightness
const uint8_t DIMMER_MAX_BRIGHTNESS = 200;

// --- Shared config -------------------------------------------------------------------------------
const std::string instanceIdFallback = "pixeldriver-trinity-1";

extern "C" void app_main() {
    initArduino();
    Serial.begin(115200);

    esp_log_level_set("wifi", ESP_LOG_WARN);
    esp_log_level_set("gpio", ESP_LOG_WARN);

    // --- Persistent storage ----------------------------------------------------------------------
    PersistentStorage::clear();
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

    // --- Pixel handlers --------------------------------------------------------------------------
    // TODO: Use proper hardware SPI instead of bit banging
    Adafruit_TLC59711 tlc59711(DIMMER_DRIVER_COUNT, DIMMER_SCLK_PIN, DIMMER_MOSI_PIN);
    auto dimmerHandler = std::make_shared<DcDimmerHandler>(tlc59711, DIMMER_CHANNEL_COUNT, DIMMER_MIN_PWM,
                                                           DIMMER_MAX_PWM, DIMMER_MAX_BRIGHTNESS);

    auto ledHandler =
        std::make_shared<SinglePinFastLedHandler<LED_PIN, RGB_ORDER>>(LED_PIXEL_COUNT, LED_MAX_BRIGHTNESS);
    while (true) {
        ESP_LOGI(TAG, "Testing LED pixels...");
        ledHandler->testPixels();
        delay(2000);
        ESP_LOGI(TAG, "Testing Dimmer Lights pixels...");

        dimmerHandler->testLights();
        delay(2000);
        break;
    }

    // --- Interfaces ------------------------------------------------------------------------------
    // Total pixel count is derived from the handlers: DC dimmer first, LED strip second
    BlockingRingBuffer<PixelFrame> artnetQueue(1);
    int totalPixelCount = dimmerHandler->pixelCount() + ledHandler->pixelCount();

    std::vector<std::shared_ptr<AbstractInterfaceHandler>> interfaces;
    interfaces.push_back(std::make_shared<ArtnetWifiHandler>(artnetQueue, totalPixelCount));

    // --- Pixel driver ----------------------------------------------------------------------------
    // Handler order defines the ArtNet stream layout: DC dimmer pixels first, LED pixels second
    PixelDriver pixelDriver(interfaces, artnetQueue, {dimmerHandler, ledHandler});
    pixelDriver.start();

    while (true) {
        delay(1000);
    }
}
