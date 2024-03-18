#include "Mcp23s17.hpp"

#include <string.h>

Mcp23s17::Mcp23s17(int vSpiMosiPin, int vSpiMisoPin, int vSpiSclkPin, int vSpiCsPin, int hSpiMosiPin, int hSpiMisoPin,
                   int hSpiSclkPin, int hSpiCsPin, unsigned int clockFrequency) {
    assert(clockFrequency <= MAX_CLOCK_FREQUENCY_HZ && "MCP23S17 clock frequency must not exceed 10 MHz");

    // TODO: Don't use MISO but only MOSI

    spi_bus_config_t vSpiBusConfig = {.mosi_io_num = vSpiMosiPin,
                                      .miso_io_num = vSpiMisoPin,
                                      .sclk_io_num = vSpiSclkPin,
                                      .quadwp_io_num = -1,
                                      .quadhd_io_num = -1,
                                      .max_transfer_sz = 0,
                                      .flags = 0};

    spi_bus_config_t hSpiBusConfig = {.mosi_io_num = hSpiMosiPin,
                                      .miso_io_num = hSpiMisoPin,
                                      .sclk_io_num = hSpiSclkPin,
                                      .quadwp_io_num = -1,
                                      .quadhd_io_num = -1,
                                      .max_transfer_sz = 0,
                                      .flags = 0};

    // Disable DMA (direct memory access) of transfer data for better performance.
    // This limits the maximum transfer size to 64 bytes which is sufficient for the MCP23S17.
    ESP_ERROR_CHECK(spi_bus_initialize(VSPI_HOST, &vSpiBusConfig, SPI_DMA_DISABLED));
    ESP_ERROR_CHECK(spi_bus_initialize(HSPI_HOST, &hSpiBusConfig, SPI_DMA_DISABLED));

    spi_device_interface_config_t vSpiDeviceConfig;
    memset(&vSpiDeviceConfig, 0, sizeof(vSpiDeviceConfig));
    vSpiDeviceConfig.spics_io_num = vSpiCsPin;
    vSpiDeviceConfig.clock_speed_hz = clockFrequency;
    vSpiDeviceConfig.mode = 0;
    vSpiDeviceConfig.queue_size = 1;

    spi_device_interface_config_t hSpiDeviceConfig;
    memset(&hSpiDeviceConfig, 0, sizeof(hSpiDeviceConfig));
    hSpiDeviceConfig.spics_io_num = hSpiCsPin;
    hSpiDeviceConfig.clock_speed_hz = clockFrequency;
    hSpiDeviceConfig.mode = 0;
    hSpiDeviceConfig.queue_size = 1;

    ESP_ERROR_CHECK(spi_bus_add_device(VSPI_HOST, &vSpiDeviceConfig, &vSpiDeviceHandle_));
    ESP_ERROR_CHECK(spi_bus_add_device(HSPI_HOST, &hSpiDeviceConfig, &hSpiDeviceHandle_));

    // Avoids the bus acquisition overhead for individual transmissions.
    // However, wile the bus is acquired, no other device can use it.
    spi_device_acquire_bus(vSpiDeviceHandle_, portMAX_DELAY);
    spi_device_acquire_bus(hSpiDeviceHandle_, portMAX_DELAY);

    // Since hardware addressing is not enabled yet, this configures all devices on the respective bus
    configureAllPinsAsOutputs(vSpiDeviceHandle_, 7);
    configureAllPinsAsOutputs(hSpiDeviceHandle_, 7);

    enableHardwareAddressing(vSpiDeviceHandle_);
    enableHardwareAddressing(hSpiDeviceHandle_);
}

void Mcp23s17::enableHardwareAddressing(spi_device_handle_t spiDeviceHandle) {
    // Enable hardware addressing for all MCP23S17 on this bus
    // Due to a silicon errata, devices with the A2 pin pulled high need to be addressed with
    // the A2 bit set even if hardware addressing is disabled.
    // Since this is the case initially, the dynamic address bits are set to 0b111 (7) for this transmission

    // writeRegister8Bit(0, IOCON_REGISTER, 1 << IOCON_HAEN_BIT);
    writeRegister8Bit(spiDeviceHandle, 7, IOCON_REGISTER, 1 << IOCON_HAEN_BIT);
}

void Mcp23s17::configureAllPinsAsOutputs(spi_device_handle_t spiDeviceHandle, uint8_t deviceId) {
    // Configure all 16 pins as outputs
    // 0 -> output
    // 1 -> input
    writeRegister16Bit(spiDeviceHandle, deviceId, IODIR_REGISTER, 0x0000);
}

void Mcp23s17::stageChannel(uint16_t channel, bool turnOn) {
    int arrayIndex = channel / 16;
    int bitIndex = channel % 16;

    if (turnOn) {
        stagedChannels_[arrayIndex] |= 1 << bitIndex;
    } else {
        stagedChannels_[arrayIndex] &= ~(1 << bitIndex);
    }
}

void Mcp23s17::commitStagedChannels() {
    // Write channels 0-15 and 16-31 in parallel
    spi_transaction_t hSpiTransaction1;
    spi_transaction_t vSpiTransaction1;
    prepareWriteTransaction16Bit(hSpiTransaction1, 0, GPIOA_REGISTER, stagedChannels_[0]);
    prepareWriteTransaction16Bit(vSpiTransaction1, 0, GPIOA_REGISTER, stagedChannels_[1]);

    spi_device_polling_start(hSpiDeviceHandle_, &hSpiTransaction1, portMAX_DELAY);
    spi_device_polling_start(vSpiDeviceHandle_, &vSpiTransaction1, portMAX_DELAY);
    spi_device_polling_end(hSpiDeviceHandle_, portMAX_DELAY);
    spi_device_polling_end(vSpiDeviceHandle_, portMAX_DELAY);

    // Write channels 32-47 and 48-63 in parallel
    spi_transaction_t hSpiTransaction2;
    spi_transaction_t vSpiTransaction2;
    prepareWriteTransaction16Bit(hSpiTransaction2, 1, GPIOA_REGISTER, stagedChannels_[2]);
    prepareWriteTransaction16Bit(vSpiTransaction2, 1, GPIOA_REGISTER, stagedChannels_[3]);

    spi_device_polling_start(hSpiDeviceHandle_, &hSpiTransaction2, portMAX_DELAY);
    spi_device_polling_start(vSpiDeviceHandle_, &vSpiTransaction2, portMAX_DELAY);
    spi_device_polling_end(hSpiDeviceHandle_, portMAX_DELAY);
    spi_device_polling_end(vSpiDeviceHandle_, portMAX_DELAY);
}

uint8_t Mcp23s17::getDeviceAddress(uint8_t deviceId) {
    assert(deviceId <= 7 && "Device ID must not exceed 7 (0b111)");
    return DEVICE_BASE_ADDRESS | deviceId;
}

void Mcp23s17::writeRegister8Bit(spi_device_handle_t spiDeviceHandle, uint8_t deviceId, uint8_t registerAddress,
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
    ESP_ERROR_CHECK(spi_device_polling_transmit(vSpiDeviceHandle_, &transaction));
}

void Mcp23s17::prepareWriteTransaction16Bit(spi_transaction_t &transaction, uint8_t deviceId, uint8_t registerAddress,
                                            uint16_t value) {
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

void Mcp23s17::writeRegister16Bit(spi_device_handle_t spiDeviceHandle, uint8_t deviceId, uint8_t registerAddress,
                                  uint16_t value) {
    spi_transaction_t transaction;
    prepareWriteTransaction16Bit(transaction, deviceId, registerAddress, value);

    // Use polling instead of interrupt transmission for better performance
    ESP_ERROR_CHECK(spi_device_polling_transmit(vSpiDeviceHandle_, &transaction));
}
