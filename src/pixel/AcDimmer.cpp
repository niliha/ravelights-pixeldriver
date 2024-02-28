#include "AcDimmer.hpp"

#include "MCP23S17.h"
#include <Arduino.h>

#include <map>

#include "PixelDriver.hpp"

namespace AcDimmer {
struct TriacEvent {
    unsigned long delayMicros;
    std::vector<int> channels;
    bool turnOn;

    TriacEvent(uint32_t delayMicros, std::vector<int> channels, bool turnOn)
        : delayMicros(delayMicros), channels(channels), turnOn(turnOn) {
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
MCP23S17 mcp(5);

volatile int currentTriacEventIndex_ = 0;
volatile int lastZeroCrossingMicros_ = 0;

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

    // Immediately switch to the triac task if it was waiting for a new event
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
            for (const auto &channel : event.channels) {
                if (event.turnOn) {
                    channelValues_ |= 1 << channel;
                } else {
                    channelValues_ &= ~(1 << channel);
                }
            }

            mcp.write16(channelValues_);
        }
        // delayMicroseconds(1); // Might be necessary to ensure the triac is turned on for at least 1 us
    }
}

int zeroCrossingCounter = 0;
int receiveDelayMicros = 0;

void init(const std::vector<int> triacPins, const int zeroCrossingPin) {
    triacPins_ = triacPins;
    zeroCrossingPin_ = zeroCrossingPin;
    eventQueue_ = xQueueCreate(triacPins.size() * 2, sizeof(int));

    ets_printf("spi begin\n");
    SPI.begin();

    ets_printf("mcp begin\n");
    assert(mcp.begin());
    ets_printf("mcp begin done\n");

    mcp.pinMode16(0x0000);
    mcp.write16(0x0000);
    ets_printf("mcp port init done\n");

    ESP_LOGI(TAG, "MOSI: %d, MISO: %d, SCK: %d, SS: %d", MOSI, MISO, SCK, SS);

    /*
    for (const int &pin : triacPins_) {
        pinMode(pin, OUTPUT);
        digitalWrite(pin, LOW);
    }
    */

    pinMode(zeroCrossingPin, INPUT_PULLUP);

    assert(getCpuFrequencyMhz() == 80 && "160 Hz and 240 Hz are currently not supported due to timer issues ");
    triacEventTimer_ = timerBegin(0, getCpuFrequencyMhz(), true);

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
    std::map<uint32_t, std::vector<int>> onChannelsByDelay;

    for (int channel = 0; channel < frame.size(); channel++) {
        uint8_t brightness = std::max({frame[channel].r, frame[channel].g, frame[channel].b});

        // Make sure TRIAC is not activated if brightness is zero
        if (brightness == 0) {
            continue;
        }

        auto triacOnDelay = map(brightness, 0, UINT8_MAX, MAX_TRIAC_ON_DELAY_MICROS_, MIN_TRIAC_ON_DELAY_MICROS_);
        onChannelsByDelay[triacOnDelay].push_back(channel);
    }

    for (auto &[onDelayMicros, channels] : onChannelsByDelay) {
        eventsBackBuffer_.emplace_back(onDelayMicros, channels, true);

        auto triacOffDelay = onDelayMicros + TRIAC_ON_DURATION_MICROS_;
        eventsBackBuffer_.emplace_back(triacOffDelay, channels, false);
    }

    // Sort the TRIAC events in the back buffer by increasing zero crossing delay
    std::sort(eventsBackBuffer_.begin(), eventsBackBuffer_.end());

    xSemaphoreGive(backBufferMutex_);

    // Signal the zero crossing ISR that the back buffer was updated
    xSemaphoreGive(backBufferUpdatedSem_);
}

}  // namespace AcDimmer