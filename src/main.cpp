#include <Arduino.h>
#define FASTLED_ESP32_I2S  // alternative parallel output driver
#include <ArtnetWifi.h>
#include <FastLED.h>
#include <array>
#include <numeric>

#include "Network.hpp"
#include "WifiCredentials.hpp"

/* BEGIN USER CONFIG */
// Specify the maximum number of pins to which lights are to be connected in a specific scenario.
// Right now this is fixed to 4.
// If a different amount must be used, setupFastled() must be adapted accordingly.
const int MAX_PIN_COUNT = 4;
// The following must be configured for the concrete setup.
// TODO: Receive lightsPerPin through artnet and store/load to/from non-volatile storage.
// Specify the amount of individually addressable pixels per "light"
const int PIXELS_PER_LIGHT = 144;
// Specify the GPIO pins to which lights are connected.
constexpr std::array<int, MAX_PIN_COUNT> PINS = {19, 18, 22, 21};
// For each pin, specify how many lights are connected.
// If there are no lights connected to a specific, set lightCount to 0.
std::array<int, MAX_PIN_COUNT> lightsPerPin = {1, 1, 1, 1};
uint8_t MAXIMUM_BRIGHTNESS = 255;
const EOrder RGB_ORDER = EOrder::RGB;
// Whether to print debug information.
const bool DO_DEBUG = false;
/* END USER CONFIG */

int PIXEL_COUNT = PIXELS_PER_LIGHT *
                  std::accumulate(lightsPerPin.begin(), lightsPerPin.end(), 0, std::plus<int>());
// The vector holding color values for each pixel.
// It must not be reallocated after setupFastLed() has been called!
std::vector<CRGB> pixels(PIXEL_COUNT);

// Artnet constants
const unsigned ARTNET_CHANNEL_COUNT = PIXEL_COUNT * 3;
const int CHANNELS_PER_UNIVERSE = 512;
// constexpr equivalent to (int) std::ceil(ARTNET_CHANNEL_COUNT / CHANNELS_PER_UNIVERSE)
const int ARTNET_UNIVERSE_COUNT =
    ARTNET_CHANNEL_COUNT / CHANNELS_PER_UNIVERSE + ((ARTNET_CHANNEL_COUNT % 512) ? 1 : 0);

std::vector<bool> universeRecvIndicators(ARTNET_UNIVERSE_COUNT, false);
bool writeFrameToLeds = false;
ArtnetWifi artnet;

// Multi-core config
TaskHandle_t fastLedTaskHandle;
TaskHandle_t artnetTaskHandle;
SemaphoreHandle_t showSemaphore = NULL;

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

void setChannel(uint16_t universeIndex, int channelIndex, uint8_t value) {
    // integer division will round off to nearest neighbor;
    unsigned pixelIndex = (universeIndex * 512 + channelIndex) / 3;
    // 0 -> first channel, 1 -> second channel, 2 -> third channel
    unsigned rgbChannelIndex = (universeIndex * 512 + channelIndex) % 3;
    if (pixelIndex >= PIXEL_COUNT) {
        return;
    }
    pixels[pixelIndex][rgbChannelIndex] = value;
}

void onDmxFrame(uint16_t universeIndex, uint16_t length, uint8_t sequence, uint8_t *data) {
    if (DO_DEBUG) {
        Serial.print("Received universe ");
        Serial.print(universeIndex);
        Serial.print(" With size ");
        Serial.println(length);
    }
    if (universeIndex >= ARTNET_UNIVERSE_COUNT) {
        if (DO_DEBUG) {
            Serial.println("Error: Received invalid universe index!");
        }
        return;
    }
    // Store which universe has got in
    universeRecvIndicators[universeIndex] = true;

    writeFrameToLeds = true;
    // Check if all universes were received yet
    for (bool isUniverseReceived : universeRecvIndicators) {
        if (!isUniverseReceived) {
            writeFrameToLeds = false;
            break;
        }
    }
    // Read universe and put into the right part of the display buffer
    for (unsigned channelIndex = 0; channelIndex < length; channelIndex++) {
        setChannel(universeIndex, channelIndex, data[channelIndex]);
    }
    // Write to leds if all universes were received
    if (writeFrameToLeds) {
        //  Reset universeReceived to 0
        std::fill(universeRecvIndicators.begin(), universeRecvIndicators.end(), false);
        // Signal to the fastled task that it can write to the leds
        xSemaphoreGive(showSemaphore);
    }
}

void setupFastLed() {
    // We can't use a loop here since addLeds() template parameters must be known at
    // compile-time
    int pixelOffset = 0;
    if (lightsPerPin[0] > 0) {
        FastLED.addLeds<WS2812, PINS[0], RGB_ORDER>(pixels.data(), pixelOffset,
                                                    lightsPerPin[0] * PIXELS_PER_LIGHT);
        pixelOffset += lightsPerPin[0] * PIXELS_PER_LIGHT;
    }
    if (lightsPerPin[1] > 0) {
        FastLED.addLeds<WS2812, PINS[1], RGB_ORDER>(pixels.data(), pixelOffset,
                                                    lightsPerPin[1] * PIXELS_PER_LIGHT);
        pixelOffset += lightsPerPin[1] * PIXELS_PER_LIGHT;
    }
    if (lightsPerPin[2] > 0) {
        FastLED.addLeds<WS2812, PINS[2], RGB_ORDER>(pixels.data(), pixelOffset,
                                                    lightsPerPin[2] * PIXELS_PER_LIGHT);
        pixelOffset += lightsPerPin[2] * PIXELS_PER_LIGHT;
    }
    if (lightsPerPin[3] > 0) {
        FastLED.addLeds<WS2812, PINS[3], RGB_ORDER>(pixels.data(), pixelOffset,
                                                    lightsPerPin[3] * PIXELS_PER_LIGHT);
        pixelOffset += lightsPerPin[3] * PIXELS_PER_LIGHT;
    }
    // Set maximum brightness (0 - 255)
    FastLED.setBrightness(MAXIMUM_BRIGHTNESS);
    // FastLED.setMaxRefreshRate(50);
}

void testLeds() {
    Serial.println("Testing LEDs...");
    std::vector<CRGB> colors{CRGB::Red, CRGB::Green, CRGB::Blue};
    for (const auto color : colors) {
        auto timeBefore = millis();
        FastLED.showColor(color);
        auto passedTime = millis() - timeBefore;
        Serial.print("show() took ");
        Serial.print(passedTime);
        Serial.println(" ms");
        delay(500);
        FastLED.clear(true);
        delay(500);
    }
    delay(2000);
}

void artnetTask(void *param) {
    Serial.print("artnetTask: started on core ");
    Serial.println(xPortGetCoreID());
    artnet.setArtDmxCallback(onDmxFrame);
    artnet.begin();
    while (true) {
        artnet.read();
    }
}

void fastledTask(void *param) {
    Serial.print("fastledTask: started on core ");
    Serial.println(xPortGetCoreID());
    while (true) {
        // Wait until artnet task signals that we can write to the leds
        xSemaphoreTake(showSemaphore, portMAX_DELAY);
        FastLED.show();
    }
}
void startTasks() {
    showSemaphore = xSemaphoreCreateBinary();  // creates a semaphore that is already taken
    if (showSemaphore == nullptr) {
        Serial.println("Error: Could not create semaphore!");
        ESP.restart();
    }

    xTaskCreatePinnedToCore(artnetTask,        /* Task function. */
                            "artnetTask",      /* name of task. */
                            10000,             /* Stack size of task */
                            NULL,              /* parameter of the task */
                            1,                 /* priority of the task */
                            &artnetTaskHandle, /* Task handle to keep track of created task */
                            0);                /* pin task to core 0 */
    delay(100);                            // giving artnetTask some time to finish its business..
    xTaskCreatePinnedToCore(fastledTask,   /* Task function. */
                            "fastledTask", /* name of task. */
                            10000,         /* Stack size of task */
                            NULL,          /* parameter of the task */
                            1,             /* priority of the task */
                            &fastLedTaskHandle, /* Task handle to keep track of created task */
                            1);                 /* pin task to core 0 */
}

void setup() {
    Serial.begin(115200);
    setupFastLed();
    testLeds();
    initNetworking();
    // Disable watch dog timer on core 0
    disableCore0WDT();
    startTasks();
}

void loop() {
}
