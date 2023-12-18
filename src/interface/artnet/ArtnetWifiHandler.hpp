#pragma once

#include <ArtnetWifi.h>

#include "AbstractArtnetHandler.hpp"

class ArtnetWifiHandler : public AbstractArtnetHandler {
 public:
    ArtnetWifiHandler(BlockingRingBuffer<PixelFrame> &frameQueue, int pixelCount);

    void start() override;
    void handleReceived() override;

 private:
    ArtnetWifi artnetWifi_;
};