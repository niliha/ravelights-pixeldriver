# ravelights-pixeldriver

This is the low-level counterpart to the [ravelights](https://github.com/danuo/ravelights) backend for driving WS28XX individually addressable LEDs or other pixel-like devices like laser diodes via an ESP32 microcontroller and [custom hardware](https://github.com/niliha/ravelights-hardware).

# Features

* Receive pixel frames via Artnet (wifi or serial)
* Discover pixel driver instances via mDNS
* Configure parallel FastLED outputs via REST API
* Drive many pixels by utilizing both CPU cores of the ESP32 and FastLED's parallel output

# Architecture

![ravelights](https://github.com/niliha/ravelights-pixeldriver/assets/75397148/61050187-542e-4873-b461-2d6d03296ba9)

# Getting started

A [PlatformIO installation](https://platformio.org/install) is required, and optionally an IDE integration, e.g. for [VSCode](https://platformio.org/install/ide?install=vscode).  
Using PlatformIO, you can build the project and upload it to an ESP32.

See `src/main.cpp` for usage and adapt the config to your setup.
