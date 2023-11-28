#include <Arduino.h>
#include <nvs_flash.h>

#include "PixelDriver.hpp"
#include "network/Network.hpp"
#include "network/WifiCredentials.hpp"

// Specify the maximum number of pins to which lights are to be connected in a specific scenario.
// Right now the PixelDriver class is fixed to 4.
const int PIN_COUNT = 4;

// Specify the amount of individually addressable pixels per "light"
const int PIXELS_PER_LIGHT = 144;

// Specify the GPIO pins to which lights are connected.
extern constexpr std::array<int, PIN_COUNT> OUTPUT_PINS = {19, 18, 22, 21};

// For each output pin, specify how many individually addressable pixels are connected.
// If there are no pixels connected to a specific, set the count to 0.
PixelOutputConfig pixelsPerOutputFallback = {1 * PIXELS_PER_LIGHT, 4 * PIXELS_PER_LIGHT, 5 * PIXELS_PER_LIGHT,
                                             6 * PIXELS_PER_LIGHT};

const EOrder RGB_ORDER = EOrder::RGB;

extern "C" void app_main() {
    // initialize arduino library before we start the tasks
    initArduino();

    Serial.begin(115200);

    // nvs_flash_erase(); // erase the NVS partition and...
    // nvs_flash_init(); // initialize the NVS partition.

    if (!Network::connectToWifi(WifiCredentials::ssid, WifiCredentials::password)) {
        ESP.restart();
    }
    // Network::initWifiAccessPoint(WifiCredentials::ssid, WifiCredentials::password);

    auto outputConfig = OutputConfigurator::loadOrApplyFallback(pixelsPerOutputFallback);

    std::vector<std::unique_ptr<AbstractNetworkInterface>> networkInterfaces;

    auto restApi = std::make_unique<RestApi>(80);
    networkInterfaces.push_back(std::move(restApi));

    BlockingRingBuffer<std::variant<PixelFrame, PixelOutputConfig>> artnetQueue(3);
    auto artnetHandler =
        std::make_unique<ArtnetHandler>(artnetQueue, outputConfig.getPixelCount(), ArtnetHandler::Mode::WIFI_ONLY);

    networkInterfaces.push_back(std::move(artnetHandler));

    PixelDriver<OUTPUT_PINS, RGB_ORDER> pixelDriver(outputConfig, std::move(networkInterfaces), artnetQueue);

    pixelDriver.testLights(PIXELS_PER_LIGHT);

    pixelDriver.start();

    while (true) {
        // Stay in setup() such that stack frame is not popped
        delay(1000);
    }
}
