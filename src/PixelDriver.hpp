#pragma once

#include <ArtnetWifi.h>
#define FASTLED_ESP32_I2S  // Alternative parallel output driver
#include <FastLED.h>
#include <numeric>
#include <set>

template <int PIN_COUNT, const std::array<int, PIN_COUNT> &PINS, EOrder RGB_ORDER = RGB>
class PixelDriver {
 public:
    PixelDriver(const std::array<int, PIN_COUNT> &lightsPerPin, int pixelsPerLight = 144,
                uint8_t maxBrightness = 255, bool debug = false)
        : MAX_BRIGHTNESS_(maxBrightness), DEBUG_(debug), PIXELS_PER_LIGHT_(pixelsPerLight),
          showSemaphore_(xSemaphoreCreateBinary()), showFinishedSem_(xSemaphoreCreateBinary()) {
        // The watchdog on core 0 is not reset anymore, since the idle task is not resumed.
        // It is disabled to avoid watchdog timeouts (resulting in a reboot).
        disableCore0WDT();
        // Improves UDP throughput drastically
        WiFi.setSleep(false);
        configure(lightsPerPin);
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

    void start() {
        assert(showSemaphore_ != nullptr && "showSemaphore_ must not be null!");
        // Start artnet task on core 0 (together with the WIFI service)
        xTaskCreatePinnedToCore(artnetTaskWrapper, "artnetTask", 4096, this, 1, NULL, 0);
        // Start the fastled task on core 1.
        // Therefore it is not affected by WIFI interrupts from core 0 while writing to the LEDs
        // (avoiding flickering).
        xTaskCreatePinnedToCore(fastledTaskWrapper, "fastledTask", 4096, this, 1, NULL, 1);
    }

 private:
    /* "Constants" */
    // The number of individually addressable pixels per "light"
    const int PIXELS_PER_LIGHT_;
    const uint8_t MAX_BRIGHTNESS_;
    const bool DEBUG_;
    // Number of required Artnet universes. Set in configure().
    int UNIVERSE_COUNT_;
    // Total number of pixels across all lights. Set in configure().
    int PIXEL_COUNT_;

    /* Data structures for FastLed and ArtnetWifi */
    // The vector holding color values for each pixel.
    // It's size is set in configure() and must not be changed afterwards!
    std::vector<CRGB> pixels_;
    std::set<int> receivedUniverses_;
    int lastUniverse_ = -1;

    // The ArtnetWifi instance
    ArtnetWifi artnet_;

    // Binary semaphore that is initialized (already taken) in initializer list.
    // It is used by the artnet task to signal the fastled task to write to the leds once all
    // universes of a single frame were received.
    SemaphoreHandle_t showSemaphore_;

    // Binary semaphore that is initialized (already taken) in initializer list.
    // It is used by the fastled task to signal the artnet task that is has finished writing to the
    // LEDs.
    SemaphoreHandle_t showFinishedSem_;

    // Used for debugging the time between frames.
    unsigned long timeOfLastFrame_ = 0;

    void configure(const std::array<int, PIN_COUNT> &lightsPerPin) {
        PIXEL_COUNT_ =
            PIXELS_PER_LIGHT_ * std::accumulate(lightsPerPin.begin(), lightsPerPin.end(), 0);
        pixels_.resize(PIXEL_COUNT_);
        UNIVERSE_COUNT_ = std::ceil((PIXEL_COUNT_ * 3) / static_cast<float>(512));
        setupFastled(lightsPerPin);
        setArtnetCallback();
    }

    void setupFastled(const std::array<int, PIN_COUNT> &lightsPerPin) {
        static_assert(PIN_COUNT == 4 && "setupFastLed() is hardcoded to handle exactly 4 pins!");
        // We can't use a loop here since addLeds() template parameters must be known at
        // compile-time
        int pixelOffset = 0;
        if (lightsPerPin[0] > 0) {
            FastLED.addLeds<WS2812, PINS[0], RGB_ORDER>(pixels_.data(), pixelOffset,
                                                        lightsPerPin[0] * PIXELS_PER_LIGHT_);
            pixelOffset += lightsPerPin[0] * PIXELS_PER_LIGHT_;
        }
        if (lightsPerPin[1] > 0) {
            FastLED.addLeds<WS2812, PINS[1], RGB_ORDER>(pixels_.data(), pixelOffset,
                                                        lightsPerPin[1] * PIXELS_PER_LIGHT_);
            pixelOffset += lightsPerPin[1] * PIXELS_PER_LIGHT_;
        }
        if (lightsPerPin[2] > 0) {
            FastLED.addLeds<WS2812, PINS[2], RGB_ORDER>(pixels_.data(), pixelOffset,
                                                        lightsPerPin[2] * PIXELS_PER_LIGHT_);
            pixelOffset += lightsPerPin[2] * PIXELS_PER_LIGHT_;
        }
        if (lightsPerPin[3] > 0) {
            FastLED.addLeds<WS2812, PINS[3], RGB_ORDER>(pixels_.data(), pixelOffset,
                                                        lightsPerPin[3] * PIXELS_PER_LIGHT_);
            pixelOffset += lightsPerPin[3] * PIXELS_PER_LIGHT_;
        }
        // Set maximum brightness (0 - 255)
        FastLED.setBrightness(MAX_BRIGHTNESS_);
        // FastLED.setMaxRefreshRate(50);
    }

    void setArtnetCallback() {
        artnet_.setArtDmxFunc(
            [this](uint16_t universeIndex, uint16_t length, uint8_t sequence, uint8_t *data) {
                this->onDmxFrame(universeIndex, length, sequence, data);
            });
    }

    void artnetTask() {
        Serial.print("artnetTask: started on core ");
        Serial.println(xPortGetCoreID());
        artnet_.begin();
        while (true) {
            uint16_t result = artnet_.read();
        }
    }

    void fastledTask() {
        Serial.print("fastledTask: started on core ");
        Serial.println(xPortGetCoreID());
        while (true) {
            // Wait until artnet task signals that we can write to the leds
            xSemaphoreTake(showSemaphore_, portMAX_DELAY);
            FastLED.show();
            xSemaphoreGive(showFinishedSem_);
        }
    }

    // Static wrappers around the task function such that we can use non-static member functions
    static void fastledTaskWrapper(void *thisPointer) {
        static_cast<PixelDriver<PIN_COUNT, PINS, RGB_ORDER> *>(thisPointer)->fastledTask();
    }

    static void artnetTaskWrapper(void *thisPointer) {
        // static_cast<PixelDriver<PIN_COUNT, PINS, RGB_ORDER> *>(_this)->artnetTask();
        static_cast<PixelDriver *>(thisPointer)->artnetTask();
    }

    void setChannel(uint16_t universeIndex, int channelIndex, uint8_t value) {
        // integer division will round off to nearest neighbor;
        unsigned pixelIndex = (universeIndex * 512 + channelIndex) / 3;
        // 0 -> first channel, 1 -> second channel, 2 -> third channel
        unsigned rgbChannelIndex = (universeIndex * 512 + channelIndex) % 3;
        if (pixelIndex >= PIXEL_COUNT_) {
            return;
        }
        pixels_[pixelIndex][rgbChannelIndex] = value;
    }

    void onDmxFrame(uint16_t universeIndex, uint16_t length, uint8_t sequence, uint8_t *data) {
        if (DEBUG_) {
            Serial.print("Received universe ");
            Serial.print(universeIndex);
            Serial.print(" With size ");
            Serial.println(length);
        }
        if (universeIndex >= UNIVERSE_COUNT_) {
            if (DEBUG_) {
                Serial.println("Error: Received invalid universe index!");
            }
            return;
        }
        if (universeIndex != (lastUniverse_ + 1) % UNIVERSE_COUNT_) {
            return;
        }
        // Store which universe has got in
        receivedUniverses_.insert(universeIndex);
        lastUniverse_ = universeIndex;
        // Read universe and put into the right part of the display buffer
        for (unsigned channelIndex = 0; channelIndex < length; channelIndex++) {
            setChannel(universeIndex, channelIndex, data[channelIndex]);
        }
        // Write to leds if all universes were received
        if (receivedUniverses_.size() == UNIVERSE_COUNT_) {
            if (DEBUG_) {
                Serial.print("Time since last frame: ");
                Serial.println(millis() - timeOfLastFrame_);
            }
            timeOfLastFrame_ = millis();
            //  Reset receivedUniverses
            receivedUniverses_.clear();
            // Signal to the fastled task that it can write to the leds
            xSemaphoreGive(showSemaphore_);
            // Wait until fastled task finished writing to the leds to avoid read/write conflict.
            xSemaphoreTake(showFinishedSem_, portMAX_DELAY);
            xSemaphoreGive(showFinishedSem_);
        }
    }
};
