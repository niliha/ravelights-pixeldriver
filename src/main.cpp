#include <ESPmDNS.h>

#include "PixelDriver.hpp"
#include "config/PersistentStorage.hpp"
#include "interface/RestApi.hpp"
#include "interface/artnet/ArtnetSerialHandler.hpp"
#include "interface/artnet/ArtnetWifiHandler.hpp"
#include "network/Network.hpp"
#include "network/WifiCredentials.hpp"
#include "pixel/AcDimmer.hpp"
#include "pixel/FastLedHandler.hpp"
#include "pixel/LaserCageHandler.hpp"

static const char *TAG = "main";

// --- Config --------------------------------------------------------------------------------------
// The GPIO pins to which lights are connected. Currently, exactly 4 pins are supported
extern constexpr std::array<int, 4> OUTPUT_PINS = {18, 19, 21, 22};

// For each of the 4 output pins, specify how many individually addressable pixels are connected.
// If there are no pixels connected to a specific pin, set the count to 0.
const int PIXELS_PER_LIGHT = 144;
OutputConfig pixelsPerOutputFallback = {1 * PIXELS_PER_LIGHT, 4 * PIXELS_PER_LIGHT, 5 * PIXELS_PER_LIGHT,
                                        6 * PIXELS_PER_LIGHT};

// The order of the R, G and B channel of the used LED strip
const EOrder RGB_ORDER = EOrder::RGB;

// The brightness scaling factor. 0 = minimum brightness, 255 = maximum brightness
const uint8_t BRIGHTNESS = 200;

// The instance ID used for mDNS discovery, must be without .local suffix
std::string instanceIdFallback = "pixeldriver-box";
// std::string instanceIdFallback = "pixeldriver-lasercage";

void dimTask(void *parameters) {
    int zeroCrossingPin = 4;
    int channelCount = 64;

    AcDimmer::init(channelCount, zeroCrossingPin);

    PixelFrame pixelFrame(channelCount);
    int maxBrightness = 20;
    int frameMillis = 100;

    while (true) {
        ESP_LOGI(TAG, "Turning lamp on slowly...");
        for (int brightness = 0; brightness <= maxBrightness; brightness++) {
            for (int channel = 0; channel < pixelFrame.size(); channel++) {
                pixelFrame[channel].r = brightness + channel;
                pixelFrame[channel].g = brightness + channel;
                pixelFrame[channel].b = brightness + channel;
            }
            AcDimmer::write(pixelFrame);
            delay(frameMillis);
        }
        delay(2000);

        ESP_LOGI(TAG, "Turning lamp off slowly...");
        for (int brightness = maxBrightness; brightness >= 0; brightness--) {
            for (int channel = 0; channel < pixelFrame.size(); channel++) {
                pixelFrame[channel].r = brightness + channel;
                pixelFrame[channel].g = brightness + channel;
                pixelFrame[channel].b = brightness + channel;
            }
            AcDimmer::write(pixelFrame);
            delay(frameMillis);
        }
        delay(2000);
    }
}
extern "C" void app_main() {
    initArduino();
    Serial.begin(115200);

    // FIXME: Just for testing
    xTaskCreatePinnedToCore(&dimTask, "dimTask", 4096, nullptr, 1, nullptr, 0);
    while (true) {
        delay(1000);
    }

    // --- Persistent storage ----------------------------------------------------------------------
    // PersistentStorage::clear();
    auto outputConfig = PersistentStorage::loadOrStoreFallbackOutputConfig(pixelsPerOutputFallback);
    auto instanceId = PersistentStorage::loadOrStoreFallbackInstanceId(instanceIdFallback);

    // --- Network ---------------------------------------------------------------------------------
    if (!Network::connectToWifi(WifiCredentials::ssid, WifiCredentials::password)) {
        ESP_LOGE(TAG, "Rebooting because connecting to wifi with SSID %s failed", WifiCredentials::ssid.c_str());
        ESP.restart();
    }

    // if(!Network::initWifiAccessPoint(WifiCredentials::ssid, WifiCredentials::password)) {
    //     ESP_LOGE(TAG, "Rebooting because setting up access point with SSID %s
    //     failed",WifiCredentials::ssid.c_str()); ESP.restart();
    // }

    if (MDNS.begin(instanceId.c_str())) {
        ESP_LOGI(TAG, "Started mDNS responder for instance id %s", instanceId.c_str());
    } else {
        ESP.restart();
    }

    // --- Interfaces ------------------------------------------------------------------------------
    std::vector<std::shared_ptr<AbstractInterfaceHandler>> interfaces;

    auto restApi = std::make_shared<RestApi>(80);
    interfaces.push_back(restApi);

    BlockingRingBuffer<PixelFrame> artnetQueue(3);
    auto artnetWifi = std::make_shared<ArtnetWifiHandler>(artnetQueue, outputConfig.getPixelCount());
    interfaces.push_back(artnetWifi);

    // auto artnetSerial = std::make_shared<ArtnetSerialHandler>(artnetQueue, outputConfig.getPixelCount());
    // interfaces.push_back(artnetSerial);

    // --- Pixel handler ---------------------------------------------------------------------------
    // LedControl ledControl(7, 6, 5);
    // LaserCageHandler pixelHandler(ledControl, outputConfig.getPixelCount());
    // pixelHandler.testLasers();

    FastLedHandler<OUTPUT_PINS, RGB_ORDER> pixelHandler(outputConfig, BRIGHTNESS);
    pixelHandler.testLights(PIXELS_PER_LIGHT);

    PixelDriver pixelDriver(interfaces, artnetQueue, pixelHandler);
    pixelDriver.start();

    while (true) {
        // Stay in app_main() such that stack frame is not popped
        delay(1000);
    }
}
