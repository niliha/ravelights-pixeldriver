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

* Clone this repository ``--recursive``ly.
* Install ESP-IDF extension for Visual Studio Code and install ESP-IDF **v5.5.1**.
See [here](https://github.com/espressif/vscode-esp-idf-extension/blob/master/docs/tutorial/install.md) for further instructions.

See the `main/examples` directory for usage examples.
A specific example can be selected for compilation by adapting `main/CmakeLists.txt`.
