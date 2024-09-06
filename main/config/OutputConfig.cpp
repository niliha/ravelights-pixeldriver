#include "OutputConfig.hpp"

#include <numeric>

int OutputConfig::getPixelCount() const {
    return std::accumulate(this->begin(), this->end(), 0);
}