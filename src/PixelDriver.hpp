#pragma once

#include <memory>

#include <esp32-hal.h>

#include "config/OutputConfig.hpp"
#include "interface/AbstractInterfaceHandler.hpp"
#include "interface/artnet/BlockingRingBuffer.hpp"
#include "pixel/AbstractPixelHandler.hpp"

class PixelDriver {
 public:
    PixelDriver(std::vector<std::shared_ptr<AbstractInterfaceHandler>> &interfaces,
                BlockingRingBuffer<PixelFrame> &artnetQueue, AbstractPixelHandler &pixelHandler);

    void start();

    static const int INTERFACE_TASK_PRIORITY_ = tskIDLE_PRIORITY;
    static const int INTERFACE_TASK_CORE_ = 0;

    static const int PIXEL_TASK_PRIORITY_ = 19;
    static const int PIXEL_TASK_CORE_ = 1;

 private:
    AbstractPixelHandler &pixelHandler_;
    std::vector<std::shared_ptr<AbstractInterfaceHandler>> &interfaces_;
    BlockingRingBuffer<PixelFrame> &artnetQueue_;

    void pixelTask();
    void interfaceTask();
};