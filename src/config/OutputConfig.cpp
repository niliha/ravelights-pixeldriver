#include "OutputConfig.hpp"

#include <numeric>


OutputConfig::OutputConfig(std::initializer_list<uint32_t> values) : std::array<uint32_t, OUTPUT_COUNT>() {
    assert(values.size() == OUTPUT_COUNT);
}

int OutputConfig::getPixelCount() const {
    return std::accumulate(this->begin(), this->end(), 0);
}

bool OutputConfig::operator==(const OutputConfig &other) const {
    return static_cast<const std::array<uint32_t, OUTPUT_COUNT> &>(*this) ==
           static_cast<const std::array<uint32_t, OUTPUT_COUNT> &>(other);
}