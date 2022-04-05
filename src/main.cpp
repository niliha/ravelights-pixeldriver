#include <Arduino.h>
#define FASTLED_ESP32_I2S  // alternative parallel output driver
#include <ArtnetWifi.h>
#include <FastLED.h>
#include <array>

#include "Network.hpp"
#include "WifiCredentials.hpp"

typedef struct Output {
    const int pinNumber;
    const int lightCount;
    const int pixelCount;
    constexpr Output(int pinNumber, int lightCount)
        : pinNumber(pinNumber), lightCount(lightCount), pixelCount(lightCount * PIXELS_PER_LIGHT) {
    }
    static const int PIXELS_PER_LIGHT = 144;
} Output;

// FastLED constants
// Specify outputs, i.e. the used GPIO pins and how many lights are connected to each pin.
// A light consists of Output::PIXELS_PER_LIGHT pixels.
// If there are no lights connected to a specific, set lightCount to 0.
constexpr Output OUTPUT1(/* pinNumber = */ 19, /* lightCount = */ 1);
constexpr Output OUTPUT2(18, 1);
constexpr Output OUTPUT3(22, 1);
constexpr Output OUTPUT4(21, 1);
const EOrder RGB_ORDER = EOrder::RGB;
constexpr int PIXEL_COUNT =
    OUTPUT1.pixelCount + OUTPUT2.pixelCount + OUTPUT3.pixelCount + OUTPUT4.pixelCount;
uint8_t MAXIMUM_BRIGHTNESS = 255;

std::array<CRGB, PIXEL_COUNT> pixels;

// Artnet constants
const unsigned ARTNET_CHANNEL_COUNT = PIXEL_COUNT * 3;
const int CHANNELS_PER_UNIVERSE = 512;
// constexpr equivalent to (int) std::ceil(ARTNET_CHANNEL_COUNT / CHANNELS_PER_UNIVERSE)
const int ARTNET_UNIVERSE_COUNT =
    ARTNET_CHANNEL_COUNT / CHANNELS_PER_UNIVERSE + ((ARTNET_CHANNEL_COUNT % 512) ? 1 : 0);

std::array<bool, ARTNET_UNIVERSE_COUNT> universeRecvIndicators;
bool writeFrameToLeds = false;
ArtnetWifi artnet;

// Multi-core config
TaskHandle_t fastLedTaskHandle;
TaskHandle_t artnetTaskHandle;
SemaphoreHandle_t showSemaphore = NULL;

const bool DO_DEBUG = true;

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
        Serial.println("Error: Received invalid universe index!");
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
        for (bool &universeRecvIndicator : universeRecvIndicators) {
            universeRecvIndicator = false;
        }
        // Signal to the fastled task that it can write to the leds
        xSemaphoreGive(showSemaphore);
    }
}

void setupFastLed() {
    // We can't use a loop here since addLeds() parameters must be known at
    // compile-time
    if (OUTPUT1.pixelCount > 0) {
        FastLED.addLeds<WS2812, OUTPUT1.pinNumber, RGB_ORDER>(pixels.data(), 0, OUTPUT1.pixelCount);
    }
    if (OUTPUT2.pixelCount > 0) {
        FastLED.addLeds<WS2812, OUTPUT2.pinNumber, RGB_ORDER>(pixels.data(), OUTPUT1.pixelCount,
                                                              OUTPUT2.pixelCount);
    }
    if (OUTPUT3.pixelCount > 0) {
        FastLED.addLeds<WS2812, OUTPUT3.pinNumber, RGB_ORDER>(
            pixels.data(), OUTPUT1.pixelCount + OUTPUT2.pixelCount, OUTPUT3.pixelCount);
    }
    if (OUTPUT4.pixelCount > 0) {
        FastLED.addLeds<WS2812, OUTPUT4.pinNumber, RGB_ORDER>(
            pixels.data(), OUTPUT1.pixelCount + OUTPUT2.pixelCount + OUTPUT3.pixelCount,
            OUTPUT4.pixelCount);
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
