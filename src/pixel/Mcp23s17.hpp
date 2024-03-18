#pragma once

#include <driver/spi_master.h>

#include <array>

class Mcp23s17 {
 public:
    Mcp23s17(int hSpiMosiPin = 13, int hSpiMisoPin = 12, int hSpiSclkPin = 14, int hSpiCsPin = 15, int vSpiMosiPin = 23,
             int vSpiMisoPin = 19, int vSpiSclkPin = 18, int vSpiCsPin = 5,
             unsigned int clockFrequency = SPI_MASTER_FREQ_10M);

    void stageChannel(uint16_t channel, bool turnOn);
    void commitStagedChannels();

 private:
    static const int MAX_CLOCK_FREQUENCY_HZ = 10 * 1000 * 1000;
    static const int DEVICE_BASE_ADDRESS = 0b00100000;

    static const uint8_t IODIR_REGISTER = 0x00;
    static const uint8_t GPIOA_REGISTER = 0x12;
    static const uint8_t IOCON_REGISTER = 0x05;

    static const uint8_t IOCON_HAEN_BIT = 3;
    std::array<uint16_t, 4> stagedChannels_;

    spi_device_handle_t hSpiDeviceHandle_;
    spi_device_handle_t vSpiDeviceHandle_;

    uint8_t getDeviceAddress(uint8_t deviceId);

    void writeRegister8Bit(spi_device_handle_t spiDeviceHandle, uint8_t deviceId, uint8_t registerAddress,
                           uint8_t value);

    void writeRegister16Bit(spi_device_handle_t spiDeviceHandle, uint8_t deviceId, uint8_t registerAddress,
                            uint16_t value);

    void prepareWriteTransaction16Bit(spi_transaction_t &transaction, uint8_t deviceId, uint8_t registerAddress,
                                      uint16_t value);

    void configureAllPinsAsOutputs(spi_device_handle_t spiDeviceHandle, uint8_t deviceId);
    void enableHardwareAddressing(spi_device_handle_t spiDeviceHandle);
};