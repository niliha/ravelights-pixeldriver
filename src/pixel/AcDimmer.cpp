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

void IRAM_ATTR onZeroCrossing() {
    // Debounce the zero crossing signal
    auto currentMicros = micros();
    if (currentMicros - lastZeroCrossingMicros_ < ZERO_CROSSING_DEBOUNCE_MICROS_) {
        return;
    }

    // Reset zero crossing interval
    timerAlarmDisable(eventTimer_);
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
        timerAlarmWrite(eventTimer_, eventsFrontBuffer_[0].delayMicros, false);
        timerAlarmEnable(eventTimer_);
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

        timerAlarmWrite(eventTimer_, nextDelayMicros, false);
        timerAlarmEnable(eventTimer_);
    }

    // Immediately switch to the triac task if it is currently waiting for a new event
    if (unblockTriacTask) {
        portYIELD_FROM_ISR();
    }
}

void IRAM_ATTR triacTask(void *param) {
    ESP_LOGI(TAG, "triacTask started on core %d", xPortGetCoreID());

    /*
    while (true) {
        portExpander_.stageChannel(0, true);
        portExpander_.stageChannel(16, true);
        portExpander_.stageChannel(32, true);
        portExpander_.stageChannel(48, true);

        auto microsBefore = micros();
        portExpander_.commitStagedChannels();
        auto passedMicros = micros() - microsBefore;
        ESP_LOGI(TAG, "commitStagedChannels took %lu µs", passedMicros);
        delay(1000);
    }
    */

    while (true) {
        uint16_t eventIndex;
        xQueueReceive(eventQueue_, &eventIndex, portMAX_DELAY);

        for (const auto &[channel, turnOn] : eventsFrontBuffer_[eventIndex].channels) {
            portExpander_.stageChannel(channel, turnOn);
        }

        portExpander_.commitStagedChannels();
    }
}

void init(const int channelCount, const int zeroCrossingPin) {
    channelCount_ = channelCount;
    zeroCrossingPin_ = zeroCrossingPin;
    // Each channel can result in at most two events (on and off) per zero crossing interval
    eventQueue_ = xQueueCreate(channelCount * 2, sizeof(uint16_t));

    pinMode(zeroCrossingPin, INPUT_PULLUP);

    eventTimer_ = timerBegin(0, 80, true);
    // FIXME: Once arduino-esp32 is v3.0.0 is released, timerAttachInterruptWithArg() can be used.
    timerAttachInterrupt(eventTimer_, &onTimerAlarm, true);
    attachInterrupt(zeroCrossingPin_, &onZeroCrossing, RISING);

    ESP_LOGI(TAG, "AcDimmer initialized with %d channels and zero crossing pin %d", channelCount_, zeroCrossingPin_);

    xTaskCreatePinnedToCore(&triacTask, "triacTask",
                            /* stack size */ 4096, nullptr, /* priority */ configMAX_PRIORITIES, NULL,
                            /* core */ 1);
}

void write(const PixelFrame &frame) {
    assert(frame.size() == channelCount_);

    std::map<uint32_t, std::vector<std::pair<int, bool>>> channelsByDelay;
    for (int channel = 0; channel < frame.size(); channel++) {
        uint8_t brightness = std::max({frame[channel].r, frame[channel].g, frame[channel].b});

        // Make sure TRIAC is not activated if brightness is zero
        if (brightness == 0) {
            continue;
        }

        auto triacOnDelay = map(brightness, 0, 255, MAX_TRIAC_EVENT_DELAY_MICROS_, MIN_TRIAC_EVENT_DELAY_MICROS_);
        channelsByDelay[triacOnDelay].emplace_back(channel, true);

        // Schedule the corresponding off event in the time slot following the on event
        auto triacOffDelay = map(brightness - 1, 0, 255, MAX_TRIAC_EVENT_DELAY_MICROS_, MIN_TRIAC_EVENT_DELAY_MICROS_);
        channelsByDelay[triacOffDelay].emplace_back(channel, false);
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

}  // namespace AcDimmer