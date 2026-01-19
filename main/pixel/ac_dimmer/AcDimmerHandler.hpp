#pragma once

#include "AbstractTriacDriver.hpp"
#include "TriacEvent.hpp"
#include "pixel/AbstractPixelHandler.hpp"
#include <Arduino.h>

class AcDimmerHandler : public AbstractPixelHandler {
 public:
    AcDimmerHandler(const int channelCount, const int zeroCrossingPin, const int triacTaskCore,
                    AbstractTriacDriver &triacDriver);
    virtual void write(const PixelFrame &frame) override;

    void testLights();

 private:
    // For 230V/50Hz, the zero crossing period is 10 ms.
    // Due to the (non-consistent) zero crossing detection delay, the minimum and maximum delay to turn
    // a TRIAC on and off must be adapted
    const int MIN_TRIAC_ON_DELAY_MICROS_ = 500;
    const int MAX_TRIAC_ON_DELAY_MICROS_ = 8800;
    const int MAX_TRIAC_OFF_DELAY_MICROS_ = 9000;
    const int ZERO_CROSSING_DEBOUNCE_MICROS_ = 1000;
    const int CHANNEL_COUNT_;
    const uint8_t MIN_BRIGHTNESS_ = 0;

    volatile unsigned long lastZeroCrossingMicros_ = 0;

    int zeroCrossingPin_;
    AbstractTriacDriver &triacDriver_;

    hw_timer_t *eventTimer_;
    QueueHandle_t eventIndexQueue_;
    volatile uint16_t currentEventIndex_ = 0;
    std::vector<TriacEvent> eventsBackBuffer_;
    std::vector<TriacEvent> eventsFrontBuffer_;
    SemaphoreHandle_t backBufferUpdatedSem_ = xSemaphoreCreateBinary();
    SemaphoreHandle_t backBufferMutex_ = xSemaphoreCreateMutex();

    void triacTask();
    void onTimerAlarm();
    void onZeroCrossing();
    void setupTimer();
    void setupZeroCrossing();
};