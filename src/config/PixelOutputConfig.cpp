#include <numeric>

#include "PixelOutputConfig.hpp"

PixelOutputConfig::PixelOutputConfig(std::initializer_list<uint32_t> values) : std::array<uint32_t, 4>() {
    if (values.size() == OUTPUT_COUNT) {
        std::copy(values.begin(), values.end(), this->begin());
    }
}
int PixelOutputConfig::getPixelCount() {
    return std::accumulate(this->begin(), this->end(), 0);
}
bool PixelOutputConfig::operator==(const PixelOutputConfig &other) const {
    return static_cast<const std::array<uint32_t, OUTPUT_COUNT> &>(*this) == static_cast<const std::array<uint32_t, 4> &>(other);
}