# Select the example to compile
set(example_main "examples/dc_dimmer/main.cpp")
# set(example_main "examples/ledstrip/main.cpp")
# set(example_main "examples/ac_dimmer/main.cpp")
# set(example_main "examples/laser/main.cpp")

set(srcs
  ${example_main}
  FpsLogger.cpp
  PixelDriver.cpp
  config/OutputConfig.cpp
  config/PersistentStorage.cpp
  config/PreferencesRaii.cpp
  interface/RestApi.cpp
  interface/artnet/AbstractArtnetHandler.cpp
  interface/artnet/ArtnetSerialHandler.cpp
  interface/artnet/ArtnetWifiHandler.cpp
  network/NetworkUtil.cpp
  pixel/LaserCageHandler.cpp
  pixel/ac_dimmer/AcDimmerHandler.cpp
  pixel/ac_dimmer/Mcp23s17TriacDriver.cpp
  pixel/dc_dimmer/DcDimmerHandler.cpp
)

idf_component_register(SRCS ${srcs} INCLUDE_DIRS ".")  
