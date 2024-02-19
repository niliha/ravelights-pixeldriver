#include "AcDimmer.hpp"

#include <Arduino.h>

namespace AcDimmer {
namespace {
struct TriacEvent {
    unsigned long delayMicros;
    int pin;
    bool turnOn;

    TriacEvent(unsigned long delayMicros, int pin, bool turnOn) : delayMicros(delayMicros), pin(pin), turnOn(turnOn) {
    }

    TriacEvent() : delayMicros(0), pin(0), turnOn(false) {
    }

    bool operator<(const TriacEvent &other) const {
        return delayMicros < other.delayMicros;
    }
};

const char *TAG = "AcDimmer";

// For 230V/50Hz, the zero crossing period is 10 ms
const int ZERO_CROSSING_PERIOD_MICROS_ = 10000;
const int ZERO_CROSSING_DEBOUNCE_MICROS_ = ZERO_CROSSING_PERIOD_MICROS_ / 10;
const int TRIAC_ON_DURATION_MICROS_ = 10;

std::vector<int> triacPins_;
int zeroCrossingPin_;

hw_timer_t *triacEventTimer_;

std::vector<TriacEvent> eventsBackBuffer_;
std::vector<TriacEvent> eventsFrontBuffer_;
SemaphoreHandle_t newBackBufferSemaphore_ = xSemaphoreCreateBinary();

int currentTriacEventIndex_ = 0;
int lastZeroCrossingMicros_ = 0;

void IRAM_ATTR onZeroCrossing() {
    auto currentMicros = micros();
    if (currentMicros - lastZeroCrossingMicros_ < ZERO_CROSSING_DEBOUNCE_MICROS_) {
        return;
    }

    if (xSemaphoreTakeFromISR(newBackBufferSemaphore_, nullptr)) {
        eventsFrontBuffer_.swap(eventsBackBuffer_);
    }

    if (eventsFrontBuffer_.empty()) {
        return;
    }

    lastZeroCrossingMicros_ = currentMicros;
    zeroCrossingCounter++;

    currentTriacEventIndex_ = 0;
    timerRestart(triacEventTimer_);

    // Schedule the first triac event. Subsequent events will be scheduled by the timer interrupt
    timerAlarmWrite(triacEventTimer_, eventsFrontBuffer_[0].delayMicros, false);
    timerAlarmEnable(triacEventTimer_);
}

void IRAM_ATTR handleTriacEvent() {
    auto currentEvent = eventsFrontBuffer_[currentTriacEventIndex_];
    digitalWrite(currentEvent.pin, currentEvent.turnOn);

    if (currentTriacEventIndex_ >= eventsFrontBuffer_.size() - 1) {
        // The last triac event was handled, thus no further events need to be scheduled
        return;
    }

    auto nextDelayMicros = eventsFrontBuffer_[currentTriacEventIndex_ + 1].delayMicros;
    currentTriacEventIndex_++;

    // Schedule the next triac event
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

    assert(getCpuFrequencyMhz() == 80 && "160 Hz and 240 Hz are not supported due to timer issues ");
    triacEventTimer_ = timerBegin(0, getCpuFrequencyMhz(), true);

    // FIXME: Once arduino-esp32 is v3.0.0 is released, timerAttachInterruptWithArg() can be used.
    timerAttachInterrupt(triacEventTimer_, &handleTriacEvent, true);
    attachInterrupt(zeroCrossingPin_, &onZeroCrossing, RISING);

    ESP_LOGI(TAG, "AcDimmer initialized with %d triac pins and zero crossing pin %d", triacPins_.size(),
             zeroCrossingPin_);
}

void write(const PixelFrame &frame) {
    assert(frame.size() == triacPins_.size());

    eventsBackBuffer_.clear();

    for (int i = 0; i < frame.size(); i++) {
        uint8_t brightness = (frame[i].r + frame[i].g + frame[i].b) / 3;

        auto triacOnDelay = map(brightness, 0, UINT8_MAX, ZERO_CROSSING_PERIOD_MICROS_ - 1000, 0);
        eventsBackBuffer_.emplace_back(triacOnDelay, triacPins_[i], true);

        auto triacOffDelay = triacOnDelay + TRIAC_ON_DURATION_MICROS_;
        eventsBackBuffer_.emplace_back(triacOffDelay, triacPins_[i], false);
    }

    std::sort(eventsBackBuffer_.begin(), eventsBackBuffer_.end());

    // Signal the zero crossing ISR that the buffer was updated
    xSemaphoreGive(newBackBufferSemaphore_);
}

}  // namespace AcDimmer
