#include "AcDimmerHandler.hpp"

#include "AcDimmer.hpp"

AcDimmerHandler::AcDimmerHandler(const int channels, const int zeroCrossingPin) {
    AcDimmer::init(channels, zeroCrossingPin);
}

void AcDimmerHandler::write(const PixelFrame &frame) {
    AcDimmer::write(frame);
}
