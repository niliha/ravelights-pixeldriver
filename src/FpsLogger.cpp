#include "FpsLogger.hpp"
#include <Arduino.h>

static const char *TAG = "FpsLogger";

FpsLogger::FpsLogger(int logIntervalSeconds) : LOG_INTERVAL_SECONDS_(logIntervalSeconds) {
}

void FpsLogger::startNewInterval(unsigned long intervalStartMillis) {
    intervalStartMillis_ = intervalStartMillis;
    lastFrameMillis = intervalStartMillis;
    frameCount_ = 0;
    minFramePeriodMillis_ = UINT32_MAX;
    maxFramePeriodMillis_ = 0;
}

void FpsLogger::notifyFrameReceived() {
    auto currentFrameMillis = millis();

    if (isFirstInterval_) {
        startNewInterval(currentFrameMillis);
        isFirstInterval_ = false;
        return;
    }

    frameCount_++;

    auto framePeriodMillis = currentFrameMillis - lastFrameMillis;
    lastFrameMillis = currentFrameMillis;
    minFramePeriodMillis_ = min(minFramePeriodMillis_, framePeriodMillis);
    maxFramePeriodMillis_ = max(maxFramePeriodMillis_, framePeriodMillis);

    auto intervalPassedMillis = currentFrameMillis - intervalStartMillis_;
    if (intervalPassedMillis >= LOG_INTERVAL_SECONDS_ * 1000) {
        ESP_LOGI(TAG, "Interval: %lu ms; Frames per second: %.2f; Frame period: (min: %lu ms, max: %lu ms)",
                 intervalPassedMillis, (float)frameCount_ / (intervalPassedMillis / 1000.0), minFramePeriodMillis_,
                 maxFramePeriodMillis_);

        startNewInterval(currentFrameMillis);
    }
}