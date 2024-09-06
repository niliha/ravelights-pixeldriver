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
    minFrameShowDurationMillis_ = UINT32_MAX;
    maxFrameShowDurationMillis_ = 0;
}

void FpsLogger::notifyFrameShown(unsigned long showDurationMillis) {
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

    minFrameShowDurationMillis_ = min(minFrameShowDurationMillis_, showDurationMillis);
    maxFrameShowDurationMillis_ = max(maxFrameShowDurationMillis_, showDurationMillis);

    auto intervalPassedMillis = currentFrameMillis - intervalStartMillis_;
    if (intervalPassedMillis >= LOG_INTERVAL_SECONDS_ * 1000) {
        ESP_LOGI(TAG,
                 "%.2f fps; "
                 "Frame period: %lu-%lu ms; "
                 "Show duration: %lu-%lu ms",
                 (float)frameCount_ / (intervalPassedMillis / 1000.0), minFramePeriodMillis_, maxFramePeriodMillis_,
                 minFrameShowDurationMillis_, maxFrameShowDurationMillis_);

        startNewInterval(currentFrameMillis);
    }
}