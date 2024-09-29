#pragma once

#include "AbstractTriacDriver.hpp"
#include <driver/spi_master.h>
#include <optional>
#include <vector>

#include <array>

class Mcp23s17TriacDriver : public AbstractTriacDriver {
 public:
    Mcp23s17TriacDriver(std::optional<std::vector<uint8_t>> customChannelMapping = std::nullopt, int spi2MosiPin = 13,
                        int spi2SclkPin = 14, int spi2CsPin = 15, int spi3MosiPin = 23, int spi3SclkPin = 18,
                        int spi3CsPin = 5, unsigned int clockFrequency = SPI_MASTER_FREQ_10M);

    virtual void stageChannel(uint16_t channel, bool turnOn) override;
    virtual void commitStagedChannels() override;

 private:
    const int MAX_CLOCK_FREQUENCY_HZ = 10 * 1000 * 1000;
    const int DEVICE_BASE_ADDRESS = 0b00100000;

    const uint8_t IODIR_REGISTER = 0x00;
    const uint8_t GPIOA_REGISTER = 0x12;
    const uint8_t IOCON_REGISTER = 0x0A;
    const uint8_t IOCON_HAEN_BIT = 3;

    const int MAX_CHANNEL_COUNT = 64;

    std::optional<std::vector<uint8_t>> customChannelMapping_;

    std::array<uint16_t, 4> stagedChannels_;
    std::array<bool, 4> isDeviceStaged_;

    spi_device_handle_t spi2DeviceHandle_;
    spi_device_handle_t spi3DeviceHandle_;

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