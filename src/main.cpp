#include "Network.hpp"
#include "PixelDriver.hpp"
#include "WifiCredentials.hpp"
#include <Arduino.h>

/* BEGIN USER CONFIG */
// Specify the maximum number of pins to which lights are to be connected in a specific scenario.
// Right now the PixelDriver class is fixed to 4.
const int MAX_PIN_COUNT = 4;
// Specify the amount of individually addressable pixels per "light"
const int PIXELS_PER_LIGHT = 144;
// Specify the GPIO pins to which lights are connected.
constexpr std::array<int, MAX_PIN_COUNT> PINS = {19, 18, 22, 21};
// For each pin, specify how many lights are connected.
// If there are no lights connected to a specific, set lightCount to 0.
// TODO: Receive lightsPerPin through artnet and store/load to/from non-volatile storage.
std::array<int, MAX_PIN_COUNT> lightsPerPin = {1, 1, 1, 1};
const EOrder RGB_ORDER = EOrder::RGB;
/* END USER CONFIG */

void initNetworking() {
    // Network::initWifiAccessPoint(WifiCredentials::ssid, WifiCredentials::password);
    if (!Network::connectToWifi(WifiCredentials::ssid, WifiCredentials::password)) {
        Serial.println("Rebooting due to wifi connection failure...");
        FastLED.showColor(CRGB::Red);  // Red indicates failure
        delay(1000);
        FastLED.clear(true);
        ESP.restart();
    } else {
        FastLED.showColor(CRGB::Green);  // Green indicates success
        delay(1000);
        FastLED.clear(true);
        delay(1000);
    }
}

void setup() {
    disableCore0WDT();
    Serial.begin(115200);

    if (!Network::connectToWifi(WifiCredentials::ssid, WifiCredentials::password)) {
        ESP.restart();
    }
    initNetworking();
    PixelDriver<MAX_PIN_COUNT, PINS, RGB_ORDER> pixelDriver(lightsPerPin, 144, 255, false);
    pixelDriver.testLeds();
    // pixelDriver.startDebug();
    pixelDriver.start();
    while (true) {
        // Stay in setup() such that stack frame is not popped
    }
}

void loop() {
}
