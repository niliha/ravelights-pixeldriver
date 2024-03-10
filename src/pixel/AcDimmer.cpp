#include "AcDimmer.hpp"

#include <Arduino.h>

#include "Mcp23s17.hpp"
#include <driver/spi_master.h>
#include <map>
#include <utility>

#include "PixelDriver.hpp"

namespace AcDimmer {
struct TriacEvent {
    unsigned long delayMicros;
    std::vector<std::pair<int, bool>> channels;

    TriacEvent(uint32_t delayMicros, std::vector<std::pair<int, bool>> channels)
        : delayMicros(delayMicros), channels(channels) {
    }

    bool operator<(const TriacEvent &other) const {
        return delayMicros < other.delayMicros;
    }
};

const char *TAG = "AcDimmer";

// For 230V/50Hz, the zero crossing period is 10 ms
const int MIN_TRIAC_EVENT_DELAY_MICROS_ = 500;
const int MAX_TRIAC_EVENT_DELAY_MICROS_ = 8800;
const int ZERO_CROSSING_DEBOUNCE_MICROS_ = 1000;
const int TRIAC_ON_DURATION_MICROS_ = 100;

std::vector<int> triacPins_;
int zeroCrossingPin_;

hw_timer_t *triacEventTimer_;

std::vector<TriacEvent> eventsBackBuffer_;
std::vector<TriacEvent> eventsFrontBuffer_;
SemaphoreHandle_t backBufferUpdatedSem_ = xSemaphoreCreateBinary();
SemaphoreHandle_t zeroCrossingDetectedSem_ = xSemaphoreCreateBinary();
SemaphoreHandle_t backBufferMutex_ = xSemaphoreCreateMutex();
QueueHandle_t eventQueue_;
volatile uint16_t channelValues_ = 0;
volatile int currentTriacEventIndex_ = 0;
volatile int lastZeroCrossingMicros_ = 0;

Mcp23s17 portExpander_;

void IRAM_ATTR onZeroCrossing() {
    // Debounce the zero crossing signal
    auto currentMicros = micros();
    if (currentMicros - lastZeroCrossingMicros_ < ZERO_CROSSING_DEBOUNCE_MICROS_) {
        return;
    }

    // Reset zero crossing interval
    // channelValues_ = 0;
    timerAlarmDisable(triacEventTimer_);
    timerRestart(triacEventTimer_);
    lastZeroCrossingMicros_ = currentMicros;
    currentTriacEventIndex_ = 0;
    zeroCrossingCounter++;

    // Swap the front and back buffer if the back buffer was updated due to write() being called
    if (xSemaphoreTakeFromISR(backBufferMutex_, nullptr)) {
        if (xSemaphoreTakeFromISR(backBufferUpdatedSem_, nullptr)) {
            eventsFrontBuffer_.swap(eventsBackBuffer_);
        }
        xSemaphoreGiveFromISR(backBufferMutex_, nullptr);
    }

    // Schedule the first TRIAC event if write() has been called yet.
    // Subsequent events will be scheduled by the timer interrupt
    if (!eventsFrontBuffer_.empty()) {
        timerAlarmWrite(triacEventTimer_, eventsFrontBuffer_[0].delayMicros, false);
        timerAlarmEnable(triacEventTimer_);
    }
}

void IRAM_ATTR onTimerAlarm() {
    // Enqueue the current event index for the triac task
    auto unblockTriacTask = pdFALSE;
    xQueueSendFromISR(eventQueue_, (const void *)&currentTriacEventIndex_, &unblockTriacTask);

    // Schedule the next triac event if there is any left
    if (currentTriacEventIndex_ < eventsFrontBuffer_.size() - 1) {
        auto nextDelayMicros = eventsFrontBuffer_[currentTriacEventIndex_ + 1].delayMicros;
        currentTriacEventIndex_++;

        timerAlarmWrite(triacEventTimer_, nextDelayMicros, false);
        timerAlarmEnable(triacEventTimer_);
    }

    // Immediately switch to the triac task if it is waiting for a new event
    if (unblockTriacTask) {
        portYIELD_FROM_ISR();
    }
}

void IRAM_ATTR triacTask(void *param) {

    ESP_LOGI(TAG, "triacTask started on core %d", xPortGetCoreID());

    while (true) {
        int eventIndex;
        if (xQueueReceive(eventQueue_, &eventIndex, portMAX_DELAY)) {
            const auto &event = eventsFrontBuffer_[eventIndex];
            for (const auto &[channel, turnOn] : event.channels) {
                if (turnOn) {
                    channelValues_ |= 1 << channel;
                } else {
                    channelValues_ &= ~(1 << channel);
                }
            }

            portExpander_.write(0, channelValues_);
        }
    }
}

int zeroCrossingCounter = 0;
int receiveDelayMicros = 0;

void init(const std::vector<int> triacPins, const int zeroCrossingPin) {
    triacPins_ = triacPins;
    zeroCrossingPin_ = zeroCrossingPin;
    eventQueue_ = xQueueCreate(triacPins.size() * 2, sizeof(int));

    pinMode(zeroCrossingPin, INPUT_PULLUP);

    triacEventTimer_ = timerBegin(0, 80, true);

    // FIXME: Once arduino-esp32 is v3.0.0 is released, timerAttachInterruptWithArg() can be used.
    timerAttachInterrupt(triacEventTimer_, &onTimerAlarm, true);
    attachInterrupt(zeroCrossingPin_, &onZeroCrossing, RISING);

    ESP_LOGI(TAG, "AcDimmer initialized with %d triac pins and zero crossing pin %d", triacPins_.size(),
             zeroCrossingPin_);

    xTaskCreatePinnedToCore(&triacTask, "triacTask",
                            /* stack size */ 4096, nullptr, /* priority */ configMAX_PRIORITIES, NULL,
                            /* core */ 1);
}

void write(const PixelFrame &frame) {
    assert(frame.size() == triacPins_.size());

    // Make sure the the back buffer is not accessed by the zero crossing ISR while it is being updated here
    xSemaphoreTake(backBufferMutex_, portMAX_DELAY);

    eventsBackBuffer_.clear();
    std::map<uint32_t, std::vector<std::pair<int, bool>>> channelsByDelay;

    for (int channel = 0; channel < frame.size(); channel++) {
        uint8_t brightness = std::max({frame[channel].r, frame[channel].g, frame[channel].b});

        // Make sure TRIAC is not activated if brightness is zero
        if (brightness == 0) {
            continue;
        }

        // Reduce brightness resolution from 8 to 7 bit to increase the delay between subsequent events
        // The last bin corresponding to the value 127 is reserved for the last possible off event
        // brightness = map(brightness, 0, 255, 1, 12);

        brightness = map(brightness, 0, 255, 1, 255);

        auto triacOnDelay = map(brightness, 0, 255, MAX_TRIAC_EVENT_DELAY_MICROS_, MIN_TRIAC_EVENT_DELAY_MICROS_);
        channelsByDelay[triacOnDelay].emplace_back(channel, true);

        // Schedule the corresponding off event in the next time slot
        auto triacOffDelay = map(brightness - 1, 0, 255, MAX_TRIAC_EVENT_DELAY_MICROS_, MIN_TRIAC_EVENT_DELAY_MICROS_);
        channelsByDelay[triacOffDelay].emplace_back(channel, false);
    }

    for (auto &[delayMicros, channels] : channelsByDelay) {
        eventsBackBuffer_.emplace_back(delayMicros, channels);
    }

    // Sort the TRIAC events in the back buffer by increasing zero crossing delay
    std::sort(eventsBackBuffer_.begin(), eventsBackBuffer_.end());

    /*
    ESP_LOGI(TAG, "**** SCHEDULE *****");
    for (auto &[delayMicros, channels] : eventsBackBuffer_) {
        for (auto &[channel, turnOn] : channels) {
            ESP_LOGI(TAG, "[%lu us] %d=%s", delayMicros, channel, turnOn ? "on" : "off");
        }
    }
    */

    xSemaphoreGive(backBufferMutex_);

    // Signal the zero crossing ISR that the back buffer was updated
    xSemaphoreGive(backBufferUpdatedSem_);
}

}  // namespace AcDimmer