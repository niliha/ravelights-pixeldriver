#include "MultiMcp23s17.hpp"

#include <string.h>

MultiMcp23s17::MultiMcp23s17(int spi2MosiPin, int spi2SclkPin, int spi2CsPin, int spi3MosiPin, int spi3SclkPin,
                             int spi3CsPin, unsigned int clockFrequency) {
    assert(clockFrequency <= MAX_CLOCK_FREQUENCY_HZ && "MCP23S17 clock frequency must not exceed 10 MHz");

    spi_bus_config_t spi2BusConfig = {.mosi_io_num = spi2MosiPin,
                                      .miso_io_num = -1,  // MISO is not needed for write operations
                                      .sclk_io_num = spi2SclkPin,
                                      .quadwp_io_num = -1,
                                      .quadhd_io_num = -1,
                                      .max_transfer_sz = 0,
                                      .flags = 0};

    spi_bus_config_t spi3BusConfig = {.mosi_io_num = spi3MosiPin,
                                      .miso_io_num = -1,  // MISO is not needed for write operations
                                      .sclk_io_num = spi3SclkPin,
                                      .quadwp_io_num = -1,
                                      .quadhd_io_num = -1,
                                      .max_transfer_sz = 0,
                                      .flags = 0};

    // Disable DMA (direct memory access) of transfer data for better performance.
    // This limits the maximum transfer size to 64 bytes which is sufficient for the MCP23S17.
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &spi2BusConfig, SPI_DMA_DISABLED));
    ESP_ERROR_CHECK(spi_bus_initialize(SPI3_HOST, &spi3BusConfig, SPI_DMA_DISABLED));

    spi_device_interface_config_t spi2DeviceConfig;
    memset(&spi2DeviceConfig, 0, sizeof(spi2DeviceConfig));
    spi2DeviceConfig.spics_io_num = spi2CsPin;
    spi2DeviceConfig.clock_speed_hz = clockFrequency;
    spi2DeviceConfig.mode = 0;
    spi2DeviceConfig.queue_size = 1;
    spi2DeviceConfig.flags = SPI_DEVICE_NO_DUMMY;

    spi_device_interface_config_t spi3DeviceConfig;
    memset(&spi3DeviceConfig, 0, sizeof(spi3DeviceConfig));
    spi3DeviceConfig.spics_io_num = spi3CsPin;
    spi3DeviceConfig.clock_speed_hz = clockFrequency;
    spi3DeviceConfig.mode = 0;
    spi3DeviceConfig.queue_size = 1;
    spi3DeviceConfig.flags = SPI_DEVICE_NO_DUMMY;

    ESP_ERROR_CHECK(spi_bus_add_device(SPI3_HOST, &spi3DeviceConfig, &spi3DeviceHandle_));
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &spi2DeviceConfig, &spi2DeviceHandle_));

    // Acquiring the bus once here avoids the bus acquisition overhead for individual transmissions.
    // However, wile the bus is acquired, no other device can use it.
    // For this reason, for each bus two MCP23S17 are treated as a single SPI device in software.
    ESP_ERROR_CHECK(spi_device_acquire_bus(spi2DeviceHandle_, portMAX_DELAY));
    ESP_ERROR_CHECK(spi_device_acquire_bus(spi3DeviceHandle_, portMAX_DELAY));

    enableHardwareAddressing(spi2DeviceHandle_);
    enableHardwareAddressing(spi3DeviceHandle_);

    configureAllPinsAsOutputs(spi2DeviceHandle_, 0);
    configureAllPinsAsOutputs(spi2DeviceHandle_, 1);
    configureAllPinsAsOutputs(spi3DeviceHandle_, 0);
    configureAllPinsAsOutputs(spi3DeviceHandle_, 1);
}

void MultiMcp23s17::enableHardwareAddressing(spi_device_handle_t spiDeviceHandle) {
    // Enable hardware addressing for all MCP23S17 on this bus.
    writeRegister8Bit(spiDeviceHandle, 0, IOCON_REGISTER, 1 << IOCON_HAEN_BIT);

    // Due to a silicon errata, devices with the A2 pin pulled high need to be addressed with
    // the A2 bit set even if hardware addressing is disabled.
    // Since this is the case initially, the dynamic address bits are set to 0b111 (7) for this transmission
    writeRegister8Bit(spiDeviceHandle, 7, IOCON_REGISTER, 1 << IOCON_HAEN_BIT);
}

void MultiMcp23s17::configureAllPinsAsOutputs(spi_device_handle_t spiDeviceHandle, uint8_t deviceId) {
    // Configure all 16 pins as outputs
    // 0 -> output
    // 1 -> input
    writeRegister16Bit(spiDeviceHandle, deviceId, IODIR_REGISTER, 0x0000);
}

void IRAM_ATTR MultiMcp23s17::stageChannel(uint16_t channel, bool turnOn) {
    assert(channel < 64 && "Channel must be in the range [0, 63]");

    int deviceIndex = channel / 16;
    int channelIndex = channel % 16;

    if (turnOn) {
        stagedChannels_[deviceIndex] |= 1 << channelIndex;
    } else {
        stagedChannels_[deviceIndex] &= ~(1 << channelIndex);
    }

    isDeviceStaged_[deviceIndex] = true;
}

void IRAM_ATTR MultiMcp23s17::commitStagedChannels() {
    // By utilizing both SPI buses, two MCP23S17 can be written to in parallel.
    // Note: Writing to all 4 MCP23S17 was measured to take around 30 µs
    // at 10 Mhz SPI and 240 MHz CPU clock frequency
    spi_transaction_t spi2Transaction;
    spi_transaction_t spi3Transaction;

    // Write channels 0-15 (MCP23S17 with A0=0 at SPI2) and 32-47 (MCP23S17 with A0=0 at SPI3) in parallel
    if (isDeviceStaged_[0]) {
        prepareWriteTransaction16Bit(spi2Transaction, 0, GPIOA_REGISTER, stagedChannels_[0]);
        ESP_ERROR_CHECK(spi_device_polling_start(spi2DeviceHandle_, &spi2Transaction, portMAX_DELAY));
    }
    if (isDeviceStaged_[2]) {
        prepareWriteTransaction16Bit(spi3Transaction, 0, GPIOA_REGISTER, stagedChannels_[2]);
        ESP_ERROR_CHECK(spi_device_polling_start(spi3DeviceHandle_, &spi3Transaction, portMAX_DELAY));
    }

    if (isDeviceStaged_[0]) {
        ESP_ERROR_CHECK(spi_device_polling_end(spi2DeviceHandle_, portMAX_DELAY));
    }
    if (isDeviceStaged_[2]) {
        ESP_ERROR_CHECK(spi_device_polling_end(spi3DeviceHandle_, portMAX_DELAY));
    }

    // Write channels 16-31 (MCP23S17 with A0=1 at SPI2) and 48-63 (MCP23S17 with A0=1 at SPI3) in parallel
    if (isDeviceStaged_[1]) {
        prepareWriteTransaction16Bit(spi2Transaction, 1, GPIOA_REGISTER, stagedChannels_[1]);
        ESP_ERROR_CHECK(spi_device_polling_start(spi2DeviceHandle_, &spi2Transaction, portMAX_DELAY));
    }
    if (isDeviceStaged_[3]) {
        prepareWriteTransaction16Bit(spi3Transaction, 1, GPIOA_REGISTER, stagedChannels_[3]);
        ESP_ERROR_CHECK(spi_device_polling_start(spi3DeviceHandle_, &spi3Transaction, portMAX_DELAY));
    }

    if (isDeviceStaged_[1]) {
        ESP_ERROR_CHECK(spi_device_polling_end(spi2DeviceHandle_, portMAX_DELAY));
    }
    if (isDeviceStaged_[3]) {
        ESP_ERROR_CHECK(spi_device_polling_end(spi3DeviceHandle_, portMAX_DELAY));
    }

    std::fill(isDeviceStaged_.begin(), isDeviceStaged_.end(), false);
}

uint8_t MultiMcp23s17::getDeviceAddress(uint8_t deviceId) {
    assert(deviceId <= 7 && "Device ID must not exceed 7 (0b111)");
    return DEVICE_BASE_ADDRESS | deviceId;
}

void MultiMcp23s17::prepareWriteTransaction16Bit(spi_transaction_t &transaction, uint8_t deviceId,
                                                 uint8_t registerAddress, uint16_t value) {
    uint8_t deviceAddress = getDeviceAddress(deviceId);

    memset(&transaction, 0, sizeof(spi_transaction_t));
    // | 0 | 1 | 0 | 0 | A2 | A1 | A0 | R/W |
    // Most significant seven bits are the shifted slave address, least significant bit is R/W select.
    // R/W == 0 -> writing, R/W == 1 -> reading
    transaction.tx_data[0] = deviceAddress << 1;
    transaction.tx_data[1] = registerAddress;
    transaction.tx_data[2] = value;
    transaction.tx_data[3] = value >> 8;
    transaction.length = 32;  // bits
    transaction.flags = SPI_TRANS_USE_TXDATA;
}

void MultiMcp23s17::writeRegister8Bit(spi_device_handle_t spiDeviceHandle, uint8_t deviceId, uint8_t registerAddress,
                                      uint8_t value) {
    uint8_t deviceAddress = getDeviceAddress(deviceId);

    spi_transaction_t transaction;
    memset(&transaction, 0, sizeof(spi_transaction_t));
    // | 0 | 1 | 0 | 0 | A2 | A1 | A0 | R/W |
    // Most significant seven bits are the shifted slave address, least significant bit is R/W select.
    // R/W == 0 -> writing, R/W == 1 -> reading
    transaction.tx_data[0] = deviceAddress << 1;
    transaction.tx_data[1] = registerAddress;
    transaction.tx_data[2] = value;
    transaction.length = 24;  // bits
    transaction.flags = SPI_TRANS_USE_TXDATA;

    // Use polling instead of interrupt transmission for better performance
    ESP_ERROR_CHECK(spi_device_polling_transmit(spiDeviceHandle, &transaction));
}

void MultiMcp23s17::writeRegister16Bit(spi_device_handle_t spiDeviceHandle, uint8_t deviceId, uint8_t registerAddress,
                                       uint16_t value) {
    spi_transaction_t transaction;
    prepareWriteTransaction16Bit(transaction, deviceId, registerAddress, value);

    // Use polling instead of interrupt transmission for better performance
    ESP_ERROR_CHECK(spi_device_polling_transmit(spiDeviceHandle, &transaction));
}
