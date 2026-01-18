# Repository Guidelines

## Project Structure & Module Organization
- `main/` holds the application code for the ESP32 pixel driver.
  - `main/pixel/` output handlers (FastLED, dimmers, laser cage).
  - `main/interface/` Artnet + REST interfaces.
  - `main/network/` WiFi and network utilities.
  - `main/config/` persistent storage and output configuration.
  - `main/examples/` sample entry points; pick one in `main/CMakeLists.txt`.
- `components/` vendored ESP-IDF components (FastLED, ArduinoJson, ArtnetWifi, etc.).
- `managed_components/` IDF-managed dependencies.
- `build/` generated artifacts.
- `sdkconfig` project configuration (ESP-IDF 5.5.1).

## Build, Test, and Development Commands
- `source ~/esp/v5.5.1/esp-idf/export.sh` activates the ESP-IDF 5.5.1 environment.
- `idf.py build` builds the firmware using ESP-IDF.
- `idf.py -p /dev/ttyUSB0 flash` flashes to a connected ESP32 (adjust port).
- `idf.py -p /dev/ttyUSB0 monitor` tails serial output.
- `idf.py menuconfig` edits `sdkconfig` values.
- `idf.py clean` clears build outputs if you need a full rebuild.

## Coding Style & Naming Conventions
- C++ code uses 4-space indentation and no tabs (see `.clang-format`).
- Run `clang-format -i main/**/*.cpp main/**/*.hpp` before commits that touch formatting.
- Naming conventions: types in `PascalCase`, methods in `lowerCamelCase`, members with trailing `_`.

## Testing Guidelines
- There is no repository-level unit test suite.
- Validate changes by building and flashing an example in `main/examples/`, then exercising the hardware.
- Always check that your solution builds at the end of the work.
- If you add new tests, document how to run them in this file.

## Commit & Pull Request Guidelines
- Commit messages are short, imperative sentences (e.g., "Limit brightness centrally via PixelDriver").
- PRs should include a concise summary, test/flash steps, and the ESP32 board + wiring used.
- Call out changes to `sdkconfig` or output configuration, and include logs/screenshots if behavior is user-visible.

## Configuration & Hardware Notes
- The default configuration lives in `sdkconfig`; update via `idf.py menuconfig` rather than editing by hand.
- REST and Artnet behavior is implemented in `main/interface/` and `main/network/`.
