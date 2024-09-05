#include "AcDimmerHandler.hpp"

#include "dimmer/AcDimmer.hpp"

AcDimmerHandler::AcDimmerHandler(const int pixelCount, const int zeroCrossingPin, const int TriacTaskCore) {
    AcDimmer::init(pixelCount, zeroCrossingPin);
}

void AcDimmerHandler::write(const PixelFrame &frame) {
    AcDimmer::write(frame);
}

void AcDimmerHandler::testLights() {
    AcDimmer::testLights();
}