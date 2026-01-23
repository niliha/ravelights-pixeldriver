#include "ArtnetWifiHandler.hpp"
#include <esp_wifi.h>

#include <ESPmDNS.h>

static const char *TAG = "ArtnetWifiHandler";

ArtnetWifiHandler::ArtnetWifiHandler(BlockingRingBuffer<PixelFrame> &frameQueue, int pixelCount)
    : AbstractArtnetHandler(frameQueue, pixelCount) {
    artnetWifi_.setArtDmxFunc([this](uint16_t universeIndex, uint16_t length, uint8_t sequence, uint8_t *data) {
        this->onDmxFrame(universeIndex, length, sequence, data);
    });

    // Improves UDP throughput drastically
    esp_err_t err = esp_wifi_set_ps(WIFI_PS_NONE);
    if (err != ESP_OK) {
        log_e("esp_wifi_set_ps failed!: 0x%x: %s", err, esp_err_to_name(err));
    }
}

void ArtnetWifiHandler::start() {
    artnetWifi_.begin();
    MDNS.addService("artnet", "udp", ART_NET_PORT);
    ESP_LOGI(TAG, "Started Artnet wifi handler on port %d", ART_NET_PORT);
}

void ArtnetWifiHandler::handleReceived() {
    // This functions calls onDmxFrame() whenever a ArtDMX packet is received
    while (artnetWifi_.read() != 0) {
        // Drain all pending packets to reduce UDP backlog.
    }
}
