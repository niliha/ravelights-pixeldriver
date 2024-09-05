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
                BlockingRingBuffer<PixelFrame> &artnetQueue, AbstractPixelHandler &pixelHandler,
                int interfaceTaskCore = 0, int interfaceTaskPriority = tskIDLE_PRIORITY, int pixelTaskCore = 1,
                int pixelTaskPriority = 19);

    void start();

 private:
    const int INTERFACE_TASK_CORE_;
    const int INTERFACE_TASK_PRIORITY_;

    const int PIXEL_TASK_CORE_;
    const int PIXEL_TASK_PRIORITY_;

    AbstractPixelHandler &pixelHandler_;
    std::vector<std::shared_ptr<AbstractInterfaceHandler>> &interfaces_;
    BlockingRingBuffer<PixelFrame> &artnetQueue_;

    void pixelTask();
    void interfaceTask();
};