#include "PixelOutputConfig.hpp"

#include <numeric>


PixelOutputConfig::PixelOutputConfig(std::initializer_list<uint32_t> values) : std::array<uint32_t, OUTPUT_COUNT>() {
    assert(values.size() == OUTPUT_COUNT);
}

int PixelOutputConfig::getPixelCount() const {
    return std::accumulate(this->begin(), this->end(), 0);
}

bool PixelOutputConfig::operator==(const PixelOutputConfig &other) const {
    return static_cast<const std::array<uint32_t, OUTPUT_COUNT> &>(*this) ==
           static_cast<const std::array<uint32_t, OUTPUT_COUNT> &>(other);
}