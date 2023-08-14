#pragma once

#include <stdint.h>
#include <vector>

/// Representation of an RGB pixel (Red, Green, Blue). Taken from FastLED's CRGB
struct Pixel {
    union {
        struct {
            union {
                uint8_t r;    ///< Red channel value
                uint8_t red;  ///< @copydoc r
            };
            union {
                uint8_t g;      ///< Green channel value
                uint8_t green;  ///< @copydoc g
            };
            union {
                uint8_t b;     ///< Blue channel value
                uint8_t blue;  ///< @copydoc b
            };
        };
        /// Access the red, green, and blue data as an array.
        /// Where:
        /// * `raw[0]` is the red value
        /// * `raw[1]` is the green value
        /// * `raw[2]` is the blue value
        uint8_t raw[3];
    };

    uint8_t& operator[] (uint8_t x) 
    {
        return raw[x];
    }
};


using PixelFrame = std::vector<Pixel>;