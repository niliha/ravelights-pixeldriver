#pragma once

#include <array>

// Don't expose OUTPUT_COUNT publicly
namespace {
constexpr int OUTPUT_COUNT = 4;
}

class PixelOutputConfig : std::array<uint32_t, OUTPUT_COUNT> {
 public:
    PixelOutputConfig(std::initializer_list<uint32_t> values);

    using std::array<uint32_t, OUTPUT_COUNT>::array;
    using std::array<uint32_t, OUTPUT_COUNT>::data;
    using std::array<uint32_t, OUTPUT_COUNT>::begin;
    using std::array<uint32_t, OUTPUT_COUNT>::end;
    using std::array<uint32_t, OUTPUT_COUNT>::size;
    using std::array<uint32_t, OUTPUT_COUNT>::operator[];

    bool operator==(const PixelOutputConfig &other) const;

    int getPixelCount() const;

    static const int OUTPUT_CONFIG_SIZE = OUTPUT_COUNT;
};