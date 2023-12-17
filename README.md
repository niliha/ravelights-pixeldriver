# ravelights-pixeldriver

This is the low-level counterpart to the [ravelights](https://github.com/danuo/ravelights) backend for driving WS28XX individually addressable LEDs or other pixel-like devices like laser diodes via an ESP32 microcontroller and [custom hardware](https://github.com/niliha/ravelights-hardware).

# Features

* Receive pixel frames via Artnet (wifi or serial)
* Discover pixel driver instances via mDNS
* Configure parallel FastLED outputs via REST API
* Can manage many pixels by utilizing both CPU cores of the ESP32


