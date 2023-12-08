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
    AbstractPixelHandler &pixelHandler_;
    std::vector<std::shared_ptr<AbstractInterfaceHandler>> &interfaces_;
    BlockingRingBuffer<PixelFrame> &artnetQueue_;

    void pixelTask();
    void interfaceTask();
};