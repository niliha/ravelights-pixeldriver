#include <nvs_flash.h>

#include "PixelDriver.hpp"
#include "config/OutputConfgurator.hpp"
#include "interface/RestApi.hpp"
#include "interface/artnet/ArtnetHandler.hpp"
#include "network/Network.hpp"
#include "network/WifiCredentials.hpp"
#include "pixel/FastLedHandler.hpp"

static const char *TAG = "main";

// Specify the maximum number of pins to which lights are to be connected in a specific scenario.
// Right now the PixelDriver class is fixed to 4.
const int PIN_COUNT = 4;

// Specify the GPIO pins to which lights are connected.
extern constexpr std::array<int, PIN_COUNT> OUTPUT_PINS = {19, 18, 22, 21};

// Specify the amount of individually addressable pixels per "light"
const int PIXELS_PER_LIGHT = 144;


// For each output pin, specify how many individually addressable pixels are connected.
// If there are no pixels connected to a specific pin, set the count to 0.
PixelOutputConfig pixelsPerOutputFallback = {1 * PIXELS_PER_LIGHT, 4 * PIXELS_PER_LIGHT, 5 * PIXELS_PER_LIGHT,
                                             6 * PIXELS_PER_LIGHT};

const EOrder RGB_ORDER = EOrder::RGB;

extern "C" void app_main() {
    initArduino();

    Serial.begin(115200);

    // nvs_flash_erase(); // erase the NVS partition and...
    // nvs_flash_init(); // initialize the NVS partition.

    if (!Network::connectToWifi(WifiCredentials::ssid, WifiCredentials::password)) {
        ESP_LOGE(TAG, "Rebooting because connecting to wifi with SSID %s failed", WifiCredentials::ssid.c_str());
        ESP.restart();
    }

    /*
    if(!Network::initWifiAccessPoint(WifiCredentials::ssid, WifiCredentials::password)) {
        ESP_LOGE(TAG, "Rebooting because setting up access point with SSID %s failed",WifiCredentials::ssid.c_str());
        ESP.restart();
    }
    */

    auto restApi = std::make_shared<RestApi>(80);

    auto outputConfig = OutputConfigurator::loadOrApplyFallback(pixelsPerOutputFallback);
    BlockingRingBuffer<PixelFrame> artnetQueue(3);
    auto artnetHandler =
        std::make_shared<ArtnetHandler>(artnetQueue, outputConfig.getPixelCount(), ArtnetHandler::Mode::WIFI_ONLY);

    std::vector<std::shared_ptr<AbstractInterfaceHandler>> networkInterfaces;
    networkInterfaces.push_back(restApi);
    networkInterfaces.push_back(artnetHandler);

    FastLedHandler<OUTPUT_PINS, RGB_ORDER> fastLedHandler(outputConfig);
    fastLedHandler.testLights(PIXELS_PER_LIGHT);

    PixelDriver pixelDriver(networkInterfaces, artnetQueue, fastLedHandler);

    pixelDriver.start();

    while (true) {
        // Stay in app_main() such that stack frame is not popped
        delay(1000);
    }
}
