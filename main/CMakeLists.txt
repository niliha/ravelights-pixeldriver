set(srcs
  main.cpp
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
  pixel/dimmer/AcDimmerHandler.cpp
  pixel/dimmer/MultiMcp23s17.cpp
)

idf_component_register(SRCS ${srcs} INCLUDE_DIRS ".")  
