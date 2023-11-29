#include "ArtnetSerial.hpp"

ArtnetSerial::ArtnetSerial(int baudRate) : onArtDmxFrame_(nullptr) {
    // Flush UART RX HW buffer to SW buffer on every received byte
    Serial2.setRxFIFOFull(1);
    // Increase SW RX buffer to hold up to 15 universes (corresponds to 16 ravelights)
    Serial2.setRxBufferSize(15 * ART_DMX_MAXIMUM_LENGTH);
    Serial2.begin(baudRate, SERIAL_8N1, UART2_RX_PIN, UART2_TX_PIN);
}

void ArtnetSerial::setArtDmxCallback(ArtDmxCallback artDmxCallback) {
    onArtDmxFrame_ = artDmxCallback;
}

void ArtnetSerial::startOver(bool success) {
    if (!success) {
        isSynced_ = false;
    }
    currentBufferIndex_ = -1;
    maximumBufferIndex_ = UINT16_MAX;
    currentState_ = State::START_DETECTION;
}

void ArtnetSerial::read() {
    if (!Serial2.available()) {
        return;
    }

    switch (currentState_) {
    case State::START_DETECTION: {
        uint8_t incomingByte = Serial2.read();

        // The next character of the Artnet header has been received
        if (incomingByte == ARTNET_HEADER[currentBufferIndex_ + 1]) {
            buffer_[++currentBufferIndex_] = incomingByte;
        } else {
            // An invalid header byte has been received for this position
            startOver(false);
            break;
        }

        // All bytes of the Artnet header (including the null terminator) have been received.
        if (currentBufferIndex_ == strlen(ARTNET_HEADER)) {
            if (!isSynced_) {
                isSynced_ = true;
                Serial.printf("(Re)-synchronized with byte stream\n");
            }
            currentState_ = State::DMX_DATA_LENGTH_PARSING;
        }
        break;
    }

    case State::OPCODE_PARSING: {
        currentBufferIndex_ +=
            Serial2.read(&buffer_[currentBufferIndex_ + 1], ARTNET_OPCODE_HI_OFFSET - currentBufferIndex_);

        if (currentBufferIndex_ == ARTNET_OPCODE_HI_OFFSET) {
            uint16_t opcode = buffer_[ARTNET_OPCODE_HI_OFFSET] << 8 | buffer_[ARTNET_OPCODE_LO_OFFSET];
            if (opcode != ART_DMX_OPCODE) {
                Serial.printf("WARN: Received unsupported Artnet opcode %x\n", opcode);
                startOver(false);
            } else {
                currentState_ = State::DMX_DATA_LENGTH_PARSING;
            }
            break;
        }

    case State::DMX_DATA_LENGTH_PARSING: {
        currentBufferIndex_ +=
            Serial2.read(&buffer_[currentBufferIndex_ + 1], ART_DMX_LENGTH_LO_OFFSET - currentBufferIndex_);

        if (currentBufferIndex_ == ART_DMX_LENGTH_LO_OFFSET) {
            dmxDataLength_ = buffer_[ART_DMX_LENGTH_HI_OFFSET] << 8 | buffer_[ART_DMX_LENGTH_LO_OFFSET];
            if (dmxDataLength_ < 2 || dmxDataLength_ > 512) {
                Serial.printf("WARN: Received invalid dmxDataLength %d. Skipping this frame\n", dmxDataLength_);
                startOver(false);
            } else {
                maximumBufferIndex_ = currentBufferIndex_ + dmxDataLength_;
                currentState_ = State::DMX_DATA_READING;
            }
        }
        break;
    }

    case State::DMX_DATA_READING: {
        currentBufferIndex_ +=
            Serial2.read(&buffer_[currentBufferIndex_ + 1], maximumBufferIndex_ - currentBufferIndex_);

        if (currentBufferIndex_ == maximumBufferIndex_) {
            uint8_t sequence = buffer_[ART_DMX_SEQUENCE_OFFSET];
            uint16_t universe = buffer_[ART_DMX_UNIVERSE_HI_OFFSET] << 8 | buffer_[ART_DMX_UNIVERSE_LO_OFFSET];
            if (onArtDmxFrame_ != nullptr) {
                onArtDmxFrame_(universe, dmxDataLength_, sequence, &buffer_[ART_DMX_DATA_OFFSET]);
            }
            startOver(true);
        }
        break;
    }
    }
    }
}