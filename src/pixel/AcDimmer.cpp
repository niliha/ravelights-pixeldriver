#include "AcDimmer.hpp"

#include <Arduino.h>

namespace AcDimmer {
namespace {
struct TriacEvent {
    uint16_t delayMicros;
    uint8_t pin;
    bool turnOn;

    TriacEvent(uint16_t delayMicros, uint8_t pin, bool turnOn) : delayMicros(delayMicros), pin(pin), turnOn(turnOn) {
    }

    TriacEvent() : delayMicros(0), pin(0), turnOn(false) {
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
SemaphoreHandle_t backBufferMutex_ = xSemaphoreCreateMutex();

int currentTriacEventIndex_ = 0;
int lastZeroCrossingMicros_ = 0;

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

    // Front buffer should only be empty if write() has not been called yet
    if (eventsFrontBuffer_.empty()) {
        return;
    }

    // Schedule the first TRIAC event. Subsequent events will be scheduled by the timer interrupt
    timerAlarmWrite(triacEventTimer_, eventsFrontBuffer_[0].delayMicros, false);
    timerAlarmEnable(triacEventTimer_);
}

void IRAM_ATTR handleTriacEvent() {
    auto currentEvent = eventsFrontBuffer_[currentTriacEventIndex_];
    if (currentEvent.turnOn) {
        digitalWrite(currentEvent.pin, HIGH);
    } else {
        digitalWrite(currentEvent.pin, LOW);
    }

    if (currentTriacEventIndex_ >= eventsFrontBuffer_.size() - 1) {
        // The last triac event was handled, thus no further events need to be scheduled
        return;
    }

    // Schedule the next triac event
    auto nextDelayMicros = eventsFrontBuffer_[currentTriacEventIndex_ + 1].delayMicros;
    currentTriacEventIndex_++;

    timerAlarmWrite(triacEventTimer_, nextDelayMicros, false);
    timerAlarmEnable(triacEventTimer_);
}

}  // namespace

int zeroCrossingCounter = 0;

void init(const std::vector<int> triacPins, const int zeroCrossingPin) {
    triacPins_ = triacPins;
    zeroCrossingPin_ = zeroCrossingPin;

    for (const int &pin : triacPins_) {
        pinMode(pin, OUTPUT);
        digitalWrite(pin, LOW);
    }
    pinMode(zeroCrossingPin, INPUT_PULLUP);

    assert(getCpuFrequencyMhz() == 80 && "160 Hz and 240 Hz are currently not supported due to timer issues ");
    triacEventTimer_ = timerBegin(0, getCpuFrequencyMhz(), true);

    // FIXME: Once arduino-esp32 is v3.0.0 is released, timerAttachInterruptWithArg() can be used.
    timerAttachInterrupt(triacEventTimer_, &handleTriacEvent, true);
    attachInterrupt(zeroCrossingPin_, &onZeroCrossing, RISING);

    ESP_LOGI(TAG, "AcDimmer initialized with %d triac pins and zero crossing pin %d", triacPins_.size(),
             zeroCrossingPin_);
}

void write(const PixelFrame &frame) {
    assert(frame.size() == triacPins_.size());

    // Make sure the the back buffer is not accessed by the zero crossing ISR while it is being updated here
    xSemaphoreTake(backBufferMutex_, portMAX_DELAY);

    eventsBackBuffer_.clear();
    for (int i = 0; i < frame.size(); i++) {
        uint8_t brightness = (frame[i].r + frame[i].g + frame[i].b) / 3;

        // Make sure TRIAC is not activated if brightness is zero
        if (brightness == 0) {
            continue;
        }

        auto triacOnDelay = map(brightness, 0, UINT8_MAX, MAX_TRIAC_ON_DELAY_MICROS_, MIN_TRIAC_ON_DELAY_MICROS_);
        eventsBackBuffer_.emplace_back(triacOnDelay, triacPins_[i], true);

        auto triacOffDelay = triacOnDelay + TRIAC_ON_DURATION_MICROS_;
        eventsBackBuffer_.emplace_back(triacOffDelay, triacPins_[i], false);
    }

    // Sort the TRIAC events in the back buffer by increasing zero crossing offset
    std::sort(eventsBackBuffer_.begin(), eventsBackBuffer_.end());

    xSemaphoreGive(backBufferMutex_);

    // Signal the zero crossing ISR that the back buffer was updated
    xSemaphoreGive(backBufferUpdatedSem_);
}

}  // namespace AcDimmer
