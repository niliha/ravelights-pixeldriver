#pragma once

#include <array>
#include <cstdint>

namespace
{
    constexpr int ARRAY_SIZE = 4;
}

class OutputConfig : public std::array<uint32_t, ARRAY_SIZE>
{
public:
    int getPixelCount() const;

    static const int OUTPUT_COUNT = ARRAY_SIZE;
    static const int FLASH_SIZE = ARRAY_SIZE * sizeof(uint32_t);
};