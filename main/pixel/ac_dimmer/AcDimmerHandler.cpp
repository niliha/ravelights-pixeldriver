#include "AcDimmerHandler.hpp"

#include <cstdint>
#include <map>
#include <numeric>

static const char *TAG = "AcDimmerHandler";

AcDimmerHandler::AcDimmerHandler(const int channelCount, const int zeroCrossingPin, const int triacTaskCore,
                                 AbstractTriacDriver &triacDriver, const uint8_t maxBrightness)
    : CHANNEL_COUNT_(channelCount), MAX_BRIGHTNESS_(maxBrightness), zeroCrossingPin_(zeroCrossingPin),
      triacDriver_(triacDriver), eventIndexQueue_(xQueueCreate(3, sizeof(uint16_t))) {

    // Initialize timer and zero crossing interrupt on the same core that will be used for the triac task
    xTaskCreatePinnedToCore(
        [](void *parameter) {
            static_cast<AcDimmerHandler *>(parameter)->setupTimer();
            static_cast<AcDimmerHandler *>(parameter)->setupZeroCrossing();
            vTaskDelete(NULL);
        },
        "initializeTimerTask", 4096, this, 0, nullptr, triacTaskCore);

    xTaskCreatePinnedToCore([](void *parameter) { static_cast<AcDimmerHandler *>(parameter)->triacTask(); },
                            "triacTask",
                            /* stack size */ 4096, this, /* priority */ configMAX_PRIORITIES - 1, NULL,
                            /* core */ triacTaskCore);

    ESP_LOGI(TAG, "AcDimmerHandler initialized on core %d with %d channels and zero crossing pin %d", xPortGetCoreID(),
             CHANNEL_COUNT_, zeroCrossingPin_);
}

std::vector<uint8_t> AcDimmerHandler::createIdentityChannelMapping(const int channelCount) const {
    std::vector<uint8_t> channelMapping(channelCount);
    std::iota(channelMapping.begin(), channelMapping.end(), 0);
    return channelMapping;
}

void AcDimmerHandler::setupTimer() {
    eventTimer_ = timerBegin(1 * 1000 * 1000);
    timerAttachInterruptArg(
        eventTimer_, [](void *parameter) { static_cast<AcDimmerHandler *>(parameter)->onTimerAlarm(); }, this);
}
void AcDimmerHandler::setupZeroCrossing() {
    pinMode(zeroCrossingPin_, INPUT);
    attachInterruptArg(
        zeroCrossingPin_, [](void *parameter) { static_cast<AcDimmerHandler *>(parameter)->onZeroCrossing(); }, this,
        RISING);
}

void AcDimmerHandler::write(const PixelFrame &frame) {
    assert(frame.size() == CHANNEL_COUNT_);

    std::map<uint32_t, std::vector<std::pair<int, bool>>> channelsByDelay;
    for (int channelIndex = 0; channelIndex < frame.size(); channelIndex++) {
        auto pixel = frame[channelIndex];
        uint8_t brightness = std::min(std::max({pixel.r, pixel.g, pixel.b}), MAX_BRIGHTNESS_);

        // Make sure TRIAC is not activated if brightness is zero
        if (brightness == 0) {
            continue;
        }

        // Map the brightness inversely to the delay to turn the TRIAC on,
        // i.e. the higher the brightness, the shorter the delay to turn the TRIAC on and vice versa
        auto triacOnDelay = map(brightness, 0, UINT8_MAX, MAX_TRIAC_ON_DELAY_MICROS_, MIN_TRIAC_ON_DELAY_MICROS_);
        channelsByDelay[triacOnDelay].emplace_back(channelIndex, true);

        // Keep the TRIAC signal on until the next zero crossing
        channelsByDelay[MAX_TRIAC_OFF_DELAY_MICROS_].emplace_back(channelIndex, false);
    }

    // Make sure the the back buffer is not accessed by the zero crossing ISR while it is being updated here
    xSemaphoreTake(backBufferMutex_, portMAX_DELAY);

    // Since the map is sorted by key, the events in the back buffer will be sorted by increasing
    // zero crossing delay
    eventsBackBuffer_.clear();
    for (auto &[delayMicros, channels] : channelsByDelay) {
        eventsBackBuffer_.emplace_back(delayMicros, channels);
    }

    xSemaphoreGive(backBufferMutex_);

    // Signal the zero crossing ISR that the back buffer was updated
    xSemaphoreGive(backBufferUpdatedSem_);
}

void AcDimmerHandler::testLights() {
    PixelFrame pixelFrame(CHANNEL_COUNT_);
    int minBrightness = 10;
    int maxBrightness = 50;
    int minBrightnessMillis = 1000;
    int synchronousFrameMillis = 50;
    int sequentialFrameMillis = 2;

    ESP_LOGI(TAG, "Fading up lights synchronously from brightness 0 to %d...", maxBrightness);
    for (int brightness = 0; brightness <= minBrightness; brightness++) {
        for (int channel = 0; channel < pixelFrame.size(); channel++) {
            pixelFrame[channel] = Pixel(brightness, brightness, brightness);
        }
        write(pixelFrame);
        delay(synchronousFrameMillis);
    }
    delay(minBrightnessMillis);

    ESP_LOGI(TAG, "Fading up lights sequentially from brightness %d to %d...", minBrightness, maxBrightness);
    for (int channel = 0; channel < pixelFrame.size(); channel++) {
        for (int brightness = minBrightness; brightness <= maxBrightness; brightness++) {
            pixelFrame[channel] = Pixel(brightness, brightness, brightness);
            write(pixelFrame);
            delay(sequentialFrameMillis);
        }
    }

    ESP_LOGI(TAG, "Fading down lights sequentially from brightness %d to %d...", maxBrightness, minBrightness);
    for (int channel = pixelFrame.size() - 1; channel >= 0; channel--) {
        for (int brightness = maxBrightness; brightness >= minBrightness; brightness--) {
            pixelFrame[channel] = Pixel(brightness, brightness, brightness);
            write(pixelFrame);
            delay(sequentialFrameMillis);
        }
    }
    delay(minBrightnessMillis);

    ESP_LOGI(TAG, "Fading down lights synchronously from brightness %d to 0...", minBrightness);
    for (int brightness = minBrightness; brightness >= 0; brightness--) {
        for (int channel = 0; channel < pixelFrame.size(); channel++) {
            pixelFrame[channel] = Pixel(brightness, brightness, brightness);
            write(pixelFrame);
        }
        write(pixelFrame);
        delay(synchronousFrameMillis);
    }
}

void IRAM_ATTR AcDimmerHandler::onZeroCrossing() {
    // Debounce the zero crossing signal
    auto currentMicros = micros();
    if (currentMicros - lastZeroCrossingMicros_ < ZERO_CROSSING_DEBOUNCE_MICROS_) {
        return;
    }

    // Reset zero crossing interval
    timerRestart(eventTimer_);
    lastZeroCrossingMicros_ = currentMicros;
    currentEventIndex_ = 0;

    // Swap the front and back buffer if the back buffer was updated due to write() being called
    if (xSemaphoreTakeFromISR(backBufferMutex_, nullptr)) {
        if (xSemaphoreTakeFromISR(backBufferUpdatedSem_, nullptr)) {
            eventsFrontBuffer_.swap(eventsBackBuffer_);
        }
        xSemaphoreGiveFromISR(backBufferMutex_, nullptr);
    }

    // Schedule the first TRIAC event if write() has been called yet.
    // Subsequent events will be scheduled by the timer ISR
    if (!eventsFrontBuffer_.empty()) {
        timerAlarm(eventTimer_, eventsFrontBuffer_[0].delayMicros, false, 0);
    }
}

void IRAM_ATTR AcDimmerHandler::onTimerAlarm() {
    // Enqueue the current event index for the triac task
    auto unblockTriacTask = pdFALSE;
    xQueueSendFromISR(eventIndexQueue_, (const void *)&currentEventIndex_, &unblockTriacTask);

    // Schedule the next triac event if there is any left
    if (currentEventIndex_ < eventsFrontBuffer_.size() - 1) {
        auto nextDelayMicros = eventsFrontBuffer_[currentEventIndex_ + 1].delayMicros;
        currentEventIndex_ = currentEventIndex_ + 1;

        timerAlarm(eventTimer_, nextDelayMicros, false, 0);
    }

    // Immediately switch to the triac task if it is currently waiting for a new event
    if (unblockTriacTask) {
        portYIELD_FROM_ISR();
    }
}

void IRAM_ATTR AcDimmerHandler::triacTask() {
    ESP_LOGI(TAG, "triacTask started on core %d", xPortGetCoreID());

    while (true) {
        uint16_t eventIndex;
        xQueueReceive(eventIndexQueue_, &eventIndex, portMAX_DELAY);

        for (const auto &[channel, turnOn] : eventsFrontBuffer_[eventIndex].channels) {
            triacDriver_.stageChannel(channel, turnOn);
        }

        triacDriver_.commitStagedChannels();
    }
}
