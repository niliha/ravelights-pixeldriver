#pragma once

#include <driver/spi_master.h>

class Mcp23s17 {
 public:
    Mcp23s17(int mosiPin = 23, int misoPin = 19, int sclkPin = 18, int csPin = 5,
             unsigned int clockFrequency = SPI_MASTER_FREQ_10M, spi_host_device_t spiHostId = SPI3_HOST);

    void write(uint8_t deviceId, uint16_t value);

 private:
    static const int MAX_CLOCK_FREQUENCY_HZ = 10 * 1000 * 1000;
    static const int DEVICE_BASE_ADDRESS = 0b00100000;

    static const uint8_t IODIR_REGISTER = 0x00;
    static const uint8_t GPIOA_REGISTER = 0x12;
    static const uint8_t IOCON_REGISTER = 0x05;

    static const uint8_t IOCON_HAEN_BIT = 3;

    spi_device_handle_t deviceHandle_;

    uint8_t getDeviceAddress(uint8_t deviceId);

    void writeRegister8Bit(uint8_t deviceAddress, uint8_t registerAddress, uint8_t value);
    void writeRegister16Bit(uint8_t deviceAddress, uint8_t registerAddress, uint16_t value);

    void configureAllPinsAsOutputs(uint8_t deviceId);
    void enableHardwareAddressing();
};