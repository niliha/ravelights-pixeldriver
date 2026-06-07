#pragma once

#include <memory>
#include <vector>

#include <esp32-hal.h>

#include "config/OutputConfig.hpp"
#include "interface/AbstractInterfaceHandler.hpp"
#include "interface/artnet/BlockingRingBuffer.hpp"
#include "pixel/AbstractPixelHandler.hpp"

class PixelDriver {
 public:
    PixelDriver(std::vector<std::shared_ptr<AbstractInterfaceHandler>> &interfaces,
                BlockingRingBuffer<PixelFrame> &artnetQueue,
                std::vector<std::shared_ptr<AbstractPixelHandler>> handlers,
                int interfaceTaskCore = 0, int interfaceTaskPriority = tskIDLE_PRIORITY,
                int pixelTaskCore = 1, int pixelTaskPriority = 19);

    void start();

 private:
    const int INTERFACE_TASK_CORE_;
    const int INTERFACE_TASK_PRIORITY_;
    const int PIXEL_TASK_CORE_;
    const int PIXEL_TASK_PRIORITY_;
    size_t TOTAL_PIXEL_COUNT_;

    std::vector<std::shared_ptr<AbstractPixelHandler>> handlers_;
    std::vector<std::shared_ptr<AbstractInterfaceHandler>> &interfaces_;
    BlockingRingBuffer<PixelFrame> &artnetQueue_;

    void pixelTask();
    void interfaceTask();

    static void adjustBrightness(std::span<Pixel> pixels, uint8_t maxBrightness);
};