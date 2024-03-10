#include "Mcp23s17.hpp"

#include <string.h>

Mcp23s17::Mcp23s17(int mosiPin, int misoPin, int sclkPin, int csPin, unsigned int clockFrequency,
                   spi_host_device_t spiHostId) {
    assert(clockFrequency <= MAX_CLOCK_FREQUENCY_HZ && "MCP23S17 clock frequency must not exceed 10 MHz");

    spi_bus_config_t busConfig = {.mosi_io_num = mosiPin,
                                  .miso_io_num = misoPin,
                                  .sclk_io_num = sclkPin,
                                  .quadwp_io_num = -1,
                                  .quadhd_io_num = -1,
                                  .max_transfer_sz = 0,
                                  .flags = 0};
    // Disable DMA (direct memory access) of transfer data for better performance.
    // This limits the maximum transfer size to 64 bytes which is sufficient for the MCP23S17.
    ESP_ERROR_CHECK(spi_bus_initialize(spiHostId, &busConfig, SPI_DMA_DISABLED));

    spi_device_interface_config_t deviceConfig;
    memset(&deviceConfig, 0, sizeof(deviceConfig));
    deviceConfig.spics_io_num = csPin;
    deviceConfig.clock_speed_hz = clockFrequency;
    deviceConfig.mode = 0;
    deviceConfig.queue_size = 1;
    ESP_ERROR_CHECK(spi_bus_add_device(spiHostId, &deviceConfig, &deviceHandle_));

    // Avoids the bus acquisition overhead for individual transmissions.
    // However, wile the bus is acquired, no other device can use it.
    spi_device_acquire_bus(deviceHandle_, portMAX_DELAY);

    // Since hardware addressing is not enabled yet, this configures all pins of all devices
    // on this bus and this chip select line as outputs
    // configureAllPinsAsOutputs(0);
    configureAllPinsAsOutputs(7);

    enableHardwareAddressing();
}

void Mcp23s17::enableHardwareAddressing() {
    // Enable hardware addressing for all MCP23S17 on this bus and this chip select line.
    // Due to a silicon errata, devices with the A2 pin pulled high need to be addressed with
    // the A2 bit set even if hardware addressing is disabled.
    // Since this is the case initially, the dynamic address bits are set to 0b111 (7) for this transmission

    // writeRegister8Bit(0, IOCON_REGISTER, 1 << IOCON_HAEN_BIT);
    writeRegister8Bit(7, IOCON_REGISTER, 1 << IOCON_HAEN_BIT);
}

void Mcp23s17::configureAllPinsAsOutputs(uint8_t deviceId) {
    // Configure all 16 pins as outputs
    // 0 -> output
    // 1 -> input
    writeRegister16Bit(deviceId, IODIR_REGISTER, 0x0000);
}

void Mcp23s17::write(uint8_t deviceId, uint16_t value) {
    writeRegister16Bit(deviceId, GPIOA_REGISTER, value);
}

uint8_t Mcp23s17::getDeviceAddress(uint8_t deviceId) {
    assert(deviceId <= 7 && "Device ID must not exceed 7 (0b111)");
    return DEVICE_BASE_ADDRESS | deviceId;
}

void Mcp23s17::writeRegister8Bit(uint8_t deviceId, uint8_t registerAddress, uint8_t value) {
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
    ESP_ERROR_CHECK(spi_device_polling_transmit(deviceHandle_, &transaction));
}

void Mcp23s17::writeRegister16Bit(uint8_t deviceId, uint8_t registerAddress, uint16_t value) {
    uint8_t deviceAddress = getDeviceAddress(deviceId);

    spi_transaction_t transaction;
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

    // Use polling instead of interrupt transmission for better performance
    ESP_ERROR_CHECK(spi_device_polling_transmit(deviceHandle_, &transaction));
}
