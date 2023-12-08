#pragma once

#include <array>

namespace {
constexpr int ARRAY_SIZE = 4;
}

class OutputConfig : std::array<uint32_t, ARRAY_SIZE> {
 public:
    OutputConfig(std::initializer_list<uint32_t> values);

    using std::array<uint32_t, ARRAY_SIZE>::array;
    using std::array<uint32_t, ARRAY_SIZE>::data;
    using std::array<uint32_t, ARRAY_SIZE>::begin;
    using std::array<uint32_t, ARRAY_SIZE>::end;
    using std::array<uint32_t, ARRAY_SIZE>::size;
    using std::array<uint32_t, ARRAY_SIZE>::operator[];

    bool operator==(const OutputConfig &other) const;

    int getPixelCount() const;

    static const int OUTPUT_COUNT = ARRAY_SIZE;
    static const int FLASH_SIZE = ARRAY_SIZE * sizeof(uint32_t);
};