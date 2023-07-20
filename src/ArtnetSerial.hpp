#pragma once

#include <Arduino.h>

typedef std::function<void(uint16_t universe, uint16_t length, uint8_t sequence, uint8_t *data)> ArtDmxCallback;

class ArtnetSerial {
 public:
    ArtnetSerial(int baudRate = 3000000);
    void setArtDmxCallback(ArtDmxCallback ArtDmxCallback);
    void read();

 private:
    static constexpr int UART2_RX_PIN = 16;
    static constexpr int UART2_TX_PIN = 17;

    static constexpr char *ARTNET_HEADER = "Art-Net";
    static constexpr int ART_DMX_OPCODE = 0x5000;
    static constexpr int ART_DMX_MAXIMUM_LENGTH = 530;

    static constexpr int ARTNET_OPCODE_LO_OFFSET = 8;
    static constexpr int ARTNET_OPCODE_HI_OFFSET = 9;
    static constexpr int ART_DMX_SEQUENCE_OFFSET = 12;
    static constexpr int ART_DMX_UNIVERSE_LO_OFFSET = 14;
    static constexpr int ART_DMX_UNIVERSE_HI_OFFSET = 15;
    static constexpr int ART_DMX_LENGTH_HI_OFFSET = 16;
    static constexpr int ART_DMX_LENGTH_LO_OFFSET = 17;
    static constexpr int ART_DMX_DATA_OFFSET = 18;

    enum class State {
        START_DETECTION,
        OPCODE_PARSING,
        DMX_DATA_LENGTH_PARSING,
        DMX_DATA_READING,
    };

    State currentState_ = State::START_DETECTION;
    uint8_t buffer_[ART_DMX_MAXIMUM_LENGTH];
    int currentBufferIndex_ = -1;
    uint16_t maximumBufferIndex_ = UINT16_MAX;
    uint16_t dmxDataLength_ = 0;
    bool isSynced_ = false;

    ArtDmxCallback onArtDmxFrame_;

    void startOver(bool success);
};