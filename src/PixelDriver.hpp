#pragma once

#include <memory>

#include "config/OutputConfig.hpp"
#include "interface/AbstractInterfaceHandler.hpp"
#include "interface/artnet/BlockingRingBuffer.hpp"
#include "pixel/AbstractPixelHandler.hpp"

class PixelDriver {
 public:
    PixelDriver(std::vector<std::shared_ptr<AbstractInterfaceHandler>> &interfaces,
                BlockingRingBuffer<PixelFrame> &artnetQueue, AbstractPixelHandler &pixelHandler);

    void start();

 private:
    static const int INTERFACE_CORE_ = 0;
    static const int PIXEL_CORE_ = 1;

    AbstractPixelHandler &pixelHandler_;
    std::vector<std::shared_ptr<AbstractInterfaceHandler>> &interfaces_;
    BlockingRingBuffer<PixelFrame> &artnetQueue_;

    void pixelTask();
    void interfaceTask();
    void disableWatchdogTimer(int core);
};