#include "ArtnetWifiHandler.hpp"

#include <ESPmDNS.h>

static const char *TAG = "ArtnetSerialHandler";

ArtnetWifiHandler::ArtnetWifiHandler(BlockingRingBuffer<PixelFrame> &frameQueue, int pixelCount)
    : AbstractArtnetHandler(frameQueue, pixelCount) {
    artnetWifi_.setArtDmxFunc([this](uint16_t universeIndex, uint16_t length, uint8_t sequence, uint8_t *data) {
        this->onDmxFrame(universeIndex, length, sequence, data);
    });
    WiFi.setSleep(false);  // Improves UDP throughput drastically
}

void ArtnetWifiHandler::start() {
    artnetWifi_.begin();
    MDNS.addService("artnet", "udp", ART_NET_PORT);
    ESP_LOGI(TAG, "Started Artnet wifi handler on port %d", ART_NET_PORT);
}

void ArtnetWifiHandler::handleReceived() {
    // This functions calls onDmxFrame() whenever a ArtDMX packet is received
    artnetWifi_.read();
}
