#pragma once

#include "MultiMcp23s17.hpp"
#include "TriacEvent.hpp"
#include "pixel/AbstractPixelHandler.hpp"
#include <Arduino.h>

class AcDimmerHandler : public AbstractPixelHandler {
 public:
    AcDimmerHandler(const int channelCount, const int zeroCrossingPin, const int triacTaskCore);
    AcDimmerHandler(const int channelCount, const int zeroCrossingPin, const int triacTaskCore,
                    const std::vector<uint8_t> &customChannelMapping);
    virtual void write(const PixelFrame &frame) override;

    void testLights();

 private:
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
    QueueHandle_t eventIndexQueue_;
    volatile uint16_t currentEventIndex_ = 0;
    std::vector<TriacEvent> eventsBackBuffer_;
    std::vector<TriacEvent> eventsFrontBuffer_;
    SemaphoreHandle_t backBufferUpdatedSem_ = xSemaphoreCreateBinary();
    SemaphoreHandle_t backBufferMutex_ = xSemaphoreCreateMutex();

    std::vector<uint8_t> channelMapping_;

    std::vector<uint8_t> createIdentityChannelMapping(const int channelCount) const;
    void triacTask();
    void onTimerAlarm();
    void onZeroCrossing();
    void setupTimer();
    void setupZeroCrossing();
};