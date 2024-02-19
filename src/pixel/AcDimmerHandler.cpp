#include "AcDimmerHandler.hpp"

#include "AcDimmer.hpp"

AcDimmerHandler::AcDimmerHandler(const std::vector<int> triacPins, const int zeroCrossingPin) {
    AcDimmer::init(triacPins, zeroCrossingPin);
}

void AcDimmerHandler::write(const PixelFrame &frame) {
    AcDimmer::write(frame);
}
