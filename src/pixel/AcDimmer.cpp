#include "AcDimmer.hpp"

#include <Arduino.h>

#include "PixelDriver.hpp"

namespace AcDimmer {
namespace {
struct TriacEvent {
    uint16_t delayMicros;
    uint8_t index;
    bool turnOn;

    TriacEvent(uint16_t delayMicros, uint8_t pin, bool turnOn) : delayMicros(delayMicros), index(pin), turnOn(turnOn) {
    }

    TriacEvent() : delayMicros(0), index(0), turnOn(false) {
    }

    bool operator<(const TriacEvent &other) const {
        return delayMicros < other.delayMicros;
    }
};

const char *TAG = "AcDimmer";

// For 230V/50Hz, the zero crossing period is 10 ms
const int MIN_TRIAC_ON_DELAY_MICROS_ = 500;
const int MAX_TRIAC_ON_DELAY_MICROS_ = 8500;
const int ZERO_CROSSING_DEBOUNCE_MICROS_ = 1000;
const int TRIAC_ON_DURATION_MICROS_ = 10;

std::vector<int> triacPins_;
int zeroCrossingPin_;

hw_timer_t *triacEventTimer_;

std::vector<TriacEvent> eventsBackBuffer_;
std::vector<TriacEvent> eventsFrontBuffer_;
SemaphoreHandle_t backBufferUpdatedSem_ = xSemaphoreCreateBinary();
SemaphoreHandle_t zeroCrossingDetectedSem_ = xSemaphoreCreateBinary();
SemaphoreHandle_t backBufferMutex_ = xSemaphoreCreateMutex();
QueueHandle_t eventQueue_;

int currentTriacEventIndex_ = 0;
volatile int lastZeroCrossingMicros_ = 0;

void IRAM_ATTR onZeroCrossing() {
    // Debounce the zero crossing signal
    auto currentMicros = micros();
    if (currentMicros - lastZeroCrossingMicros_ < ZERO_CROSSING_DEBOUNCE_MICROS_) {
        return;
    }

    // Reset zero crossing interval
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
    auto currentEvent = eventsFrontBuffer_[currentTriacEventIndex_];

    // Send the current event to the triac task
    auto unblockTriacTask = pdFALSE;
    xQueueSendFromISR(eventQueue_, &currentEvent, &unblockTriacTask);

    // Schedule the next triac event if there is any left
    if (currentTriacEventIndex_ < eventsFrontBuffer_.size() - 1) {
        auto nextDelayMicros = eventsFrontBuffer_[currentTriacEventIndex_ + 1].delayMicros;
        currentTriacEventIndex_++;

        timerAlarmWrite(triacEventTimer_, nextDelayMicros, false);
        timerAlarmEnable(triacEventTimer_);
    }

    // Immediately switch to the triac task if it was waiting for a new event
    if (unblockTriacTask) {
        portYIELD_FROM_ISR();
    }
}

void triacTask(void *param) {
    ESP_LOGI(TAG, "triacTask started on core %d", xPortGetCoreID());

    while (true) {
        TriacEvent event;
        xQueueReceive(eventQueue_, &event, portMAX_DELAY);

        digitalWrite(triacPins_[event.index], event.turnOn ? HIGH : LOW);
    }
}

}  // namespace

int zeroCrossingCounter = 0;

void init(const std::vector<int> triacPins, const int zeroCrossingPin) {
    triacPins_ = triacPins;
    zeroCrossingPin_ = zeroCrossingPin;
    eventQueue_ = xQueueCreate(triacPins.size() * 2, sizeof(TriacEvent));

    for (const int &pin : triacPins_) {
        pinMode(pin, OUTPUT);
        digitalWrite(pin, LOW);
    }
    pinMode(zeroCrossingPin, INPUT_PULLUP);

    assert(getCpuFrequencyMhz() == 80 && "160 Hz and 240 Hz are currently not supported due to timer issues ");
    triacEventTimer_ = timerBegin(0, getCpuFrequencyMhz(), true);

    // FIXME: Once arduino-esp32 is v3.0.0 is released, timerAttachInterruptWithArg() can be used.
    timerAttachInterrupt(triacEventTimer_, &onTimerAlarm, true);
    attachInterrupt(zeroCrossingPin_, &onZeroCrossing, RISING);

    ESP_LOGI(TAG, "AcDimmer initialized with %d triac pins and zero crossing pin %d", triacPins_.size(),
             zeroCrossingPin_);

    xTaskCreatePinnedToCore(&triacTask, "triacTask",
                            /* stack size */ 4096, nullptr, /* priority */ PixelDriver::PIXEL_TASK_PRIORITY_, NULL,
                            /* core */ PixelDriver::PIXEL_TASK_CORE_);
}

void write(const PixelFrame &frame) {
    assert(frame.size() == triacPins_.size());

    // Make sure the the back buffer is not accessed by the zero crossing ISR while it is being updated here
    xSemaphoreTake(backBufferMutex_, portMAX_DELAY);

    eventsBackBuffer_.clear();
    for (int i = 0; i < frame.size(); i++) {
        uint8_t brightness = max(max(frame[i].r, frame[i].g), frame[i].b);

        // Make sure TRIAC is not activated if brightness is zero
        if (brightness == 0) {
            continue;
        }

        auto triacOnDelay = map(brightness, 0, UINT8_MAX, MAX_TRIAC_ON_DELAY_MICROS_, MIN_TRIAC_ON_DELAY_MICROS_);
        eventsBackBuffer_.emplace_back(triacOnDelay, triacPins_[i], true);

        auto triacOffDelay = triacOnDelay + TRIAC_ON_DURATION_MICROS_;
        eventsBackBuffer_.emplace_back(triacOffDelay, triacPins_[i], false);
    }

    // Sort the TRIAC events in the back buffer by increasing zero crossing delay
    std::sort(eventsBackBuffer_.begin(), eventsBackBuffer_.end());

    xSemaphoreGive(backBufferMutex_);

    // Signal the zero crossing ISR that the back buffer was updated
    xSemaphoreGive(backBufferUpdatedSem_);
}

}  // namespace AcDimmer
