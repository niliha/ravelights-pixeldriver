#pragma once

class FpsLogger {
 public:
    FpsLogger(int logIntervalSeconds=5);
    void notifyFrameShown(unsigned long showDurationMillis);


 private:
    const int LOG_INTERVAL_SECONDS_;

    bool isFirstInterval_= true;

    unsigned long frameCount_;
    unsigned long intervalStartMillis_;
    unsigned long lastFrameMillis;
    unsigned long minFramePeriodMillis_;
    unsigned long maxFramePeriodMillis_;

    unsigned long minFrameShowDurationMillis_;
    unsigned long maxFrameShowDurationMillis_;

    void startNewInterval(unsigned long intervalStartMillis);


};