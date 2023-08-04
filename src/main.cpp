#include "Network.hpp"
#include "PixelDriver.hpp"
#include "WifiCredentials.hpp"
#include <Arduino.h>
    #include <nvs_flash.h>

/* BEGIN USER CONFIG */
// Specify the maximum number of pins to which lights are to be connected in a specific scenario.
// Right now the PixelDriver class is fixed to 4.
const int MAX_PIN_COUNT = 4;
// Specify the amount of individually addressable pixels per "light"
const int PIXELS_PER_LIGHT = 144;
// Specify the GPIO pins to which lights are connected.
// extern constexpr std::array<int, MAX_PIN_COUNT> PINS = {19, 18, 22, 21};
extern constexpr std::array<int, MAX_PIN_COUNT> PINS = {19};
// For each pin, specify how many lights are connected.
// If there are no lights connected to a specific, set lightCount to 0.
// TODO: Receive lightsPerPin through artnet and store/load to/from non-volatile storage.
// std::array<int, MAX_PIN_COUNT> lightsPerPin = {1, 1, 1, 1};
std::array<uint8_t, MAX_PIN_COUNT> lightsPerPin = {5,0,0,0};
const EOrder RGB_ORDER = EOrder::RGB;
/* END USER CONFIG */

void setup() {
    Serial.begin(115200);

     // nvs_flash_erase(); // erase the NVS partition and...
     // nvs_flash_init(); // initialize the NVS partition.

    if (!Network::connectToWifi(WifiCredentials::ssid, WifiCredentials::password)) {
        ESP.restart();
    }
    // Network::initWifiAccessPoint(WifiCredentials::ssid, WifiCredentials::password);

    PixelDriver<MAX_PIN_COUNT, PINS, RGB_ORDER> pixelDriver(lightsPerPin, PIXELS_PER_LIGHT);
    pixelDriver.testLeds();
    pixelDriver.start();

    while (true) {
        // Stay in setup() such that stack frame is not popped
        delay(1000);
    }
}

void loop() {
}
