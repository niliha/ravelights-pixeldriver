#include "AcDimmer.hpp"

#include <Arduino.h>
#include <map>

#include "MultiMcp23s17.hpp"

namespace AcDimmer {

struct TriacEvent {
    unsigned long delayMicros;
    std::vector<std::pair<int, bool>> channels;

    TriacEvent(uint32_t delayMicros, std::vector<std::pair<int, bool>> channels)
        : delayMicros(delayMicros), channels(channels) {
    }
};

const char *TAG = "AcDimmer";

// For 230V/50Hz, the zero crossing period is 10 ms.
// Due to the (non-consistent) zero crossing detection delay, the minimum and maximum delay to turn
// a TRIAC on and off must be adapted
const int MIN_TRIAC_EVENT_DELAY_MICROS_ = 500;
const int MAX_TRIAC_EVENT_DELAY_MICROS_ = 8800;
const int ZERO_CROSSING_DEBOUNCE_MICROS_ = 1000;

volatile unsigned long lastZeroCrossingMicros_ = 0;

int channelCount_;
int zeroCrossingPin_;
MultiMcp23s17 portExpander_;

hw_timer_t *eventTimer_;
QueueHandle_t eventQueue_;
volatile uint16_t currenEventIndex_ = 0;
std::vector<TriacEvent> eventsBackBuffer_;
std::vector<TriacEvent> eventsFrontBuffer_;
SemaphoreHandle_t backBufferUpdatedSem_ = xSemaphoreCreateBinary();
SemaphoreHandle_t backBufferMutex_ = xSemaphoreCreateMutex();

std::vector<uint8_t> channelMapping_;

void IRAM_ATTR onZeroCrossing() {
    // Debounce the zero crossing signal
    auto currentMicros = micros();
    if (currentMicros - lastZeroCrossingMicros_ < ZERO_CROSSING_DEBOUNCE_MICROS_) {
        return;
    }

    // Reset zero crossing interval
    timerRestart(eventTimer_);
    lastZeroCrossingMicros_ = currentMicros;
    currenEventIndex_ = 0;

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

void IRAM_ATTR onTimerAlarm() {
    // Enqueue the current event index for the triac task
    auto unblockTriacTask = pdFALSE;
    xQueueSendFromISR(eventQueue_, (const void *)&currenEventIndex_, &unblockTriacTask);

    // Schedule the next triac event if there is any left
    if (currenEventIndex_ < eventsFrontBuffer_.size() - 1) {
        auto nextDelayMicros = eventsFrontBuffer_[currenEventIndex_ + 1].delayMicros;
        currenEventIndex_++;

        timerAlarm(eventTimer_, nextDelayMicros, false, 0);
    }

    // Immediately switch to the triac task if it is currently waiting for a new event
    if (unblockTriacTask) {
        portYIELD_FROM_ISR();
    }
}

void IRAM_ATTR triacTask(void *param) {
    ESP_LOGI(TAG, "triacTask started on core %d", xPortGetCoreID());

    while (true) {
        uint16_t eventIndex;
        xQueueReceive(eventQueue_, &eventIndex, portMAX_DELAY);

        for (const auto &[channel, turnOn] : eventsFrontBuffer_[eventIndex].channels) {
            portExpander_.stageChannel(channel, turnOn);
        }

        portExpander_.commitStagedChannels();
    }
}

void init(const int channelCount, const int zeroCrossingPin, const int triacTaskCore) {
    channelCount_ = channelCount;
    zeroCrossingPin_ = zeroCrossingPin;
    // Each channel can result in at most two events (on and off) per zero crossing interval
    eventQueue_ = xQueueCreate(channelCount * 2, sizeof(uint16_t));

    // FIXME: Add proper way for custom channel mapping
    // clang-format off
    channelMapping_.assign({
        0, 1,  2,  3,  4,  5,  6,   // Klein A
        7, 8,  9,  10, 11, 12, 13,  // Klein B
        21, 22, 23, 24, 25, 26, 27, // Mittel B
        28, 29, 30, 31, 32, 33, 34, // Mittel C
        14, 15, 16, 17, 18, 19, 20, // Mittel A
        42, 43, 44, 45, 46, 47, 48, // Groß B
        35, 36, 37, 38, 39, 40, 41, // Groß A
        56, 57, 58, 59, 60, 61, 62 , // Groß D
        49, 50, 51, 52, 53, 54, 55, // Groa C
    });
    // clang-format on
    assert(channelMapping_.size() == channelCount);

    pinMode(zeroCrossingPin, INPUT);

    // Initialize timer on the same core that will be used for the triac task
    xTaskCreatePinnedToCore(
        [](void *parameters) {
            eventTimer_ = timerBegin(1 * 1000 * 1000 /* 1 Mhz */);
            // FIXME: Once arduino-esp32 is v3.0.0 is released, timerAttachInterruptWithArg() can be used.
            timerAttachInterrupt(eventTimer_, &onTimerAlarm);
            attachInterrupt(zeroCrossingPin_, &onZeroCrossing, RISING);

            ESP_LOGI(TAG, "AcDimmer initialized on core %d with %d channels and zero crossing pin %d", xPortGetCoreID(),
                     channelCount_, zeroCrossingPin_);

            vTaskDelete(NULL);
        },
        "testLightsTask", 4096, nullptr, 0, nullptr, triacTaskCore);

    xTaskCreatePinnedToCore(&triacTask, "triacTask",
                            /* stack size */ 4096, nullptr, /* priority */ configMAX_PRIORITIES, NULL,
                            /* core */ triacTaskCore);
}

void write(const PixelFrame &frame) {
    assert(frame.size() == channelCount_);

    std::map<uint32_t, std::vector<std::pair<int, bool>>> channelsByDelay;
    for (int channelIndex = 0; channelIndex < frame.size(); channelIndex++) {
        uint8_t brightness = std::max({frame[channelMapping_[channelIndex]].r, frame[channelMapping_[channelIndex]].g,
                                       frame[channelMapping_[channelIndex]].b});

        // Make sure TRIAC is not activated if brightness is zero
        if (brightness == 0) {
            continue;
        }

        auto triacOnDelay = map(brightness, 0, 255, MAX_TRIAC_EVENT_DELAY_MICROS_, MIN_TRIAC_EVENT_DELAY_MICROS_);
        channelsByDelay[triacOnDelay].emplace_back(channelIndex, true);

        // Schedule the corresponding off event in the time slot following the on event
        auto triacOffDelay = map(brightness - 1, 0, 255, MAX_TRIAC_EVENT_DELAY_MICROS_, MIN_TRIAC_EVENT_DELAY_MICROS_);
        channelsByDelay[triacOffDelay].emplace_back(channelIndex, false);
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

void testLights() {
    PixelFrame pixelFrame(channelCount_);
    int maxBrightness = 70;
    int frameMillis = 2;
    ESP_LOGI(TAG, "Turning on lights slowly...");
    for (int channel = 0; channel < pixelFrame.size(); channel++) {
        for (int brightness = 0; brightness <= maxBrightness; brightness++) {
            pixelFrame[channel].r = brightness;
            pixelFrame[channel].g = brightness;
            pixelFrame[channel].b = brightness;
            AcDimmer::write(pixelFrame);
            delay(frameMillis);
        }
    }

    ESP_LOGI(TAG, "Turning off lights slowly...");
    for (int channel = pixelFrame.size() - 1; channel >= 0; channel--) {
        for (int brightness = maxBrightness; brightness >= 0; brightness--) {
            pixelFrame[channel].r = brightness;
            pixelFrame[channel].g = brightness;
            pixelFrame[channel].b = brightness;
            AcDimmer::write(pixelFrame);
            delay(frameMillis);
        }
    }
}

}  // namespace AcDimmer