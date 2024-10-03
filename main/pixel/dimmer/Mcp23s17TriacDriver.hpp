#pragma once

#include "AbstractTriacDriver.hpp"
#include <array>
#include <driver/spi_master.h>
#include <optional>

class Mcp23s17TriacDriver : public AbstractTriacDriver {
 public:
    static constexpr int MAX_CHANNEL_COUNT = 64;

    Mcp23s17TriacDriver(std::optional<std::array<uint8_t, MAX_CHANNEL_COUNT>> customChannelMapping = std::nullopt,
                        int spi2MosiPin = 13, int spi2SclkPin = 14, int spi2CsPin = 15, int spi3MosiPin = 23,
                        int spi3SclkPin = 18, int spi3CsPin = 5, unsigned int clockFrequency = SPI_MASTER_FREQ_10M);

    virtual void stageChannel(uint16_t channel, bool turnOn) override;
    virtual void commitStagedChannels() override;

 private:
    static constexpr int MAX_CLOCK_FREQUENCY_HZ = 10 * 1000 * 1000;
    static constexpr int DEVICE_BASE_ADDRESS = 0b00100000;

    static constexpr uint8_t IODIR_REGISTER = 0x00;
    static constexpr uint8_t GPIOA_REGISTER = 0x12;
    static constexpr uint8_t IOCON_REGISTER = 0x0A;
    static constexpr uint8_t IOCON_HAEN_BIT = 3;

    static constexpr int MAX_DEVICE_COUNT = 4;

    std::optional<std::array<uint8_t, MAX_CHANNEL_COUNT>> customChannelMapping_;
    std::array<uint16_t, MAX_DEVICE_COUNT> stagedChannels_;
    std::array<bool, MAX_DEVICE_COUNT> isDeviceStaged_;

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