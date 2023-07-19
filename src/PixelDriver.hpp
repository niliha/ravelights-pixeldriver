#pragma once

#include <ArtnetWifi.h>
#define FASTLED_ESP32_I2S  // Alternative parallel output driver
#include <FastLED.h>
#include <FrameQueue.hpp>
#include <memory>
#include <mutex>
#include <numeric>
#include <set>
#include <string.h>

template <int PIN_COUNT, const std::array<int, PIN_COUNT> &PINS, EOrder RGB_ORDER = RGB> class PixelDriver {
 public:
    PixelDriver(const std::array<int, PIN_COUNT> &lightsPerPin, float framesPerSecond = 20, int frameQueueCapacity = 3,
                int pixelsPerLight = 144, bool debug = false)
        : DEBUG_(debug), PIXELS_PER_LIGHT_(pixelsPerLight), FRAME_PERIOD_MS_(1000 / framesPerSecond),
          artnetFrame_(std::make_shared<std::vector<CRGB>>()), frameQueue_(frameQueueCapacity) {

        configure(lightsPerPin);
    }

    void testLeds() {
        Serial.println("Testing LEDs...");
        std::vector<CRGB> colors{CRGB::Red, CRGB::Green, CRGB::Blue};
        for (const auto color : colors) {
            auto millisBefore = millis();
            FastLED.showColor(color);
            auto passedMillis = millis() - millisBefore;
            Serial.printf("show() took %d ms\n", passedMillis);
            delay(500);
            FastLED.clear(true);
            delay(500);
        }
        delay(2000);
    }

    void start() {
        // Start artnet task on core 0 (together with the WIFI service)
        xTaskCreatePinnedToCore(staticArtnetTask, "artnetTask", 4096, this, 1, NULL, 0);

        // Start the fastled task on core 1.
        // Therefore it is not affected by WIFI interrupts from core 0 while writing to the LEDs
        // (avoiding flickering).
        xTaskCreatePinnedToCore(staticFastledTask, "fastledTask", 4096, this, 1, NULL, 1);
    }

 private:
    // Whether to print debug information
    const bool DEBUG_;

    // The number of individually addressable pixels per ravelight
    const int PIXELS_PER_LIGHT_;

    // The period in milliseconds to wait between writing subsequent frames to the LEDs.
    const int FRAME_PERIOD_MS_;

    // Total number of pixels across all lights. Set in configure().
    int pixelCount_;

    // The vector holding color values for each pixel.
    // It's size is set in configure() and must not be changed afterwards!
    std::vector<CRGB> fastLedPixels_;

    // The ArtnetWifi instance
    ArtnetWifi artnet_;

    // Number of required Artnet universes. Set in configure().
    int universeCount_;

    // Set of universe indices indicating which universes of the current frame have been received yet
    std::set<int> receivedUniverses_;

    // Queue for storing incoming Artnet frames
    FrameQueue<std::shared_ptr<std::vector<CRGB>>> frameQueue_;

    // Mutex for synchronizing access to the frame queue by the ArtNet and FastLED task
    std::mutex queue_mutex_;

    // Pointer referencing the frame currently filled by the ArtNet task
    std::shared_ptr<std::vector<CRGB>> artnetFrame_;

    // Used for debugging the time between frames.
    unsigned long lastFrameMillis_ = 0;

    void configure(const std::array<int, PIN_COUNT> &lightsPerPin) {
        // The watchdog on core 0 is not reset anymore, since the idle task is not resumed.
        // It is disabled to avoid watchdog timeouts (resulting in a reboot).
        disableCore0WDT();

        // Improves UDP throughput drastically
        WiFi.setSleep(false);

        pixelCount_ = PIXELS_PER_LIGHT_ * std::accumulate(lightsPerPin.begin(), lightsPerPin.end(), 0);
        fastLedPixels_.resize(pixelCount_);
        artnetFrame_->resize(pixelCount_);
        universeCount_ = std::ceil((pixelCount_ * 3) / static_cast<float>(512));

        setupFastled(lightsPerPin);
        setArtnetCallback();

        Serial.begin(115200);
        Serial2.setRxFIFOFull(1);
        Serial2.setRxBufferSize(4096);
        // Serial2.begin(460800, SERIAL_8N1, 16, 17);
        Serial2.begin(921600, SERIAL_8N1, 16, 17);
    }

    void setupFastled(const std::array<int, PIN_COUNT> &lightsPerPin) {
        static_assert(PIN_COUNT == 4, "setupFastLed() is hardcoded to handle exactly 4 pins!");
        // We can't use a loop here since addLeds() template parameters must be known at
        // compile-time
        int pixelOffset = 0;
        if (lightsPerPin[0] > 0) {
            FastLED.addLeds<WS2812, PINS[0], RGB_ORDER>(fastLedPixels_.data(), pixelOffset,
                                                        lightsPerPin[0] * PIXELS_PER_LIGHT_);
            pixelOffset += lightsPerPin[0] * PIXELS_PER_LIGHT_;
        }
        if (lightsPerPin[1] > 0) {
            FastLED.addLeds<WS2812, PINS[1], RGB_ORDER>(fastLedPixels_.data(), pixelOffset,
                                                        lightsPerPin[1] * PIXELS_PER_LIGHT_);
            pixelOffset += lightsPerPin[1] * PIXELS_PER_LIGHT_;
        }
        if (lightsPerPin[2] > 0) {
            FastLED.addLeds<WS2812, PINS[2], RGB_ORDER>(fastLedPixels_.data(), pixelOffset,
                                                        lightsPerPin[2] * PIXELS_PER_LIGHT_);
            pixelOffset += lightsPerPin[2] * PIXELS_PER_LIGHT_;
        }
        if (lightsPerPin[3] > 0) {
            FastLED.addLeds<WS2812, PINS[3], RGB_ORDER>(fastLedPixels_.data(), pixelOffset,
                                                        lightsPerPin[3] * PIXELS_PER_LIGHT_);
            pixelOffset += lightsPerPin[3] * PIXELS_PER_LIGHT_;
        }
        FastLED.setCorrection(TypicalLEDStrip);
    }

    void setArtnetCallback() {
        artnet_.setArtDmxFunc([this](uint16_t universeIndex, uint16_t length, uint8_t sequence, uint8_t *data) {
            this->onDmxFrame(universeIndex, length, sequence, data);
        });
    }

    void fastledTask() {
        Serial.printf("fastledTask: started on core %d\n", xPortGetCoreID());

        while (true) {
            auto millisBefore = millis();

            // Update pixel buffer if there is a new frame in the queue
            {
                // Aquire lock and release it at end of this scope
                std::lock_guard<std::mutex> lock(queue_mutex_);

                if (!frameQueue_.isEmpty()) {
                    // Copy oldest frame from the queue into pixel buffer
                    fastLedPixels_ = *frameQueue_.front();
                    // Pop pointer to oldest frame from the queue
                    frameQueue_.pop();

                    // Write pixel buffer to the LEDs
                    FastLED.show();
                }
            }

            // Wait until a frame period has passed
            auto passedMillis = millis() - millisBefore;
            auto millisToWait = FRAME_PERIOD_MS_ - passedMillis;
            if (millisToWait < 0) {
                Serial.printf("WARN: FPS to high! Would need to wait negative time: %d ms", millisToWait);
                continue;
            }
            // delay(passedMillis);
        }
    }

    static void staticFastledTask(void *thisPointer) {
        static_cast<PixelDriver *>(thisPointer)->fastledTask();
    }

    void artnetTask() {
        Serial.printf("artnetTask: started on core %d\n", xPortGetCoreID());

        artnet_.begin();

        while (true) {
            // This calls onDmxFrame() whenever a ArtDMX packet is received
             artnet_.read();

            parseArtnetPacketOverSerial();
        }
    }

    static void staticArtnetTask(void *thisPointer) {
        static_cast<PixelDriver *>(thisPointer)->artnetTask();
    }

    void onDmxFrame(uint16_t universeIndex, uint16_t length, uint8_t sequence, uint8_t *data) {
        if (DEBUG_) {
            // Serial.printf("Received universe %d with %d bytes\n", universeIndex, length);
        }

        if (universeIndex >= universeCount_) {
            if (DEBUG_) {
                Serial.printf("WARN: Received invalid universe index %d\n", universeIndex);
            }
            return;
        }

        // Store which universe has got in
        receivedUniverses_.insert(universeIndex);

        // Read universe and put into the right part of the display buffer
        for (unsigned channelIndex = 0; channelIndex < length; channelIndex++) {
            setChannel(universeIndex, channelIndex, data[channelIndex]);
        }

        // All data for this frame has been received
        if (receivedUniverses_.size() == universeCount_) {
            if (DEBUG_) {
                Serial.printf("Time since last frame: %d ms\n", millis() - lastFrameMillis_);
            }
            lastFrameMillis_ = millis();

            // Reset information about received universes
            receivedUniverses_.clear();

            // Put the filled frame into the queue such that the FasLED task can consume it
            {
                std::lock_guard<std::mutex> lock(queue_mutex_);
                frameQueue_.push(artnetFrame_);

                // Create a new vector for the next frame.
                artnetFrame_ = std::make_shared<std::vector<CRGB>>(pixelCount_);
            }
        }
    }

    void setChannel(uint16_t universeIndex, int channelIndex, uint8_t value) {
        // integer division will round off to nearest neighbor;
        unsigned pixelIndex = (universeIndex * 512 + channelIndex) / 3;
        // 0 -> first channel, 1 -> second channel, 2 -> third channel
        unsigned rgbChannelIndex = (universeIndex * 512 + channelIndex) % 3;
        if (pixelIndex >= pixelCount_) {
            return;
        }
        (*artnetFrame_)[pixelIndex][rgbChannelIndex] = value;
    }

    enum class ArtnetOverSerialState { START_DETECTION, METADATA_PARSING, DMX_DATA_READING };

    void parseArtnetPacketOverSerial() {
        const char *ARTNET_HEADER = "Art-Net";

        static uint8_t buffer[1024];
        static int currentBufferIndex = -1;
        static auto currentState = ArtnetOverSerialState::START_DETECTION;
        static uint16_t maximumBufferIndex = UINT16_MAX;
        static uint16_t dmxDataLength = 0;
        static unsigned long packet_start_millis = 0;

        if (!Serial2.available()) {
            // Nothing available to read. Try again next time
            return;
        }

        // Serial.printf("Read byte %c\n", incomingByte);

        switch (currentState) {
        case ArtnetOverSerialState::START_DETECTION: {

            uint8_t incomingByte = Serial2.read();
            // The next character of the Artnet header has been received.
            if (incomingByte == ARTNET_HEADER[currentBufferIndex + 1]) {
                buffer[++currentBufferIndex] = incomingByte;
                // Serial.printf("byte %d (%c) of header has been received\n", currentBufferIndex, incomingByte);
                // if (currentBufferIndex == 0) {
                //     packet_start_millis = millis();
                // }
            } else {
                // An invalid artnet header byte has been received. Start over from the beginning
                // Serial.printf("Invalid artnet header byte (%c) for index %d. Starting all over...\n", incomingByte,
                //               currentBufferIndex + 1);
                currentBufferIndex = -1;
                break;
            }
            // All bytes of the Artnet header (including the null terminator) have been received. Switch to parsing mode
            if (currentBufferIndex == strlen(ARTNET_HEADER)) {
                //Serial.printf("Header has been Received. Switching to parsing mode...\n");
                currentState = ArtnetOverSerialState::METADATA_PARSING;
            }
            break;
        }
        case ArtnetOverSerialState::METADATA_PARSING:
            //printf("Trying to read %d remaining bytes for metadata\n", 17 - currentBufferIndex);
            currentBufferIndex += Serial2.read(buffer + currentBufferIndex + 1, 17 - currentBufferIndex);
            //printf("Now at index %d\n", currentBufferIndex);        

            if (currentBufferIndex == 17) {
                dmxDataLength = buffer[17] | buffer[16] << 8;
                if (dmxDataLength < 2 || dmxDataLength > 512) {
                    Serial.printf("WARN: Received invalid dmxDataLength %d. Skipping this frame\n", dmxDataLength);
                    currentBufferIndex = -1;
                    maximumBufferIndex = UINT16_MAX;
                    currentState = ArtnetOverSerialState::START_DETECTION;
                    break;
                }
                maximumBufferIndex = currentBufferIndex + dmxDataLength;
                //Serial.printf("Continuing to read %d DMX bytes until index %d\n", dmxDataLength, maximumBufferIndex);
                currentState = ArtnetOverSerialState::DMX_DATA_READING;
                break;
            }
            break;

        case ArtnetOverSerialState::DMX_DATA_READING:
            currentBufferIndex +=
                Serial2.read(buffer + currentBufferIndex + 1, maximumBufferIndex - currentBufferIndex);

            if (currentBufferIndex == maximumBufferIndex) {
                uint8_t sequence = buffer[12];
                uint16_t universe = buffer[14] | buffer[15] << 8;
                onDmxFrame(universe, dmxDataLength, sequence, buffer + 18);
                //Serial.printf("Received complete universe %d with %d bytes\n. Switching back to WAITING mode\n",
                //              universe, dmxDataLength);
                //Serial.printf("Last buffer index was %d\n", currentBufferIndex);
                // Serial.printf("Receiving packet took %d ms\n", millis() - packet_start_millis);
                currentBufferIndex = -1;
                maximumBufferIndex = UINT16_MAX;
                currentState = ArtnetOverSerialState::START_DETECTION;
                break;
            }
            break;
        }
    }
};
