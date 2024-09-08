#include "ArtnetSerialHandler.hpp"

#include <HardwareSerial.h>

static const char *TAG = "ArtnetSerialHandler";

ArtnetSerialHandler::ArtnetSerialHandler(BlockingRingBuffer<PixelFrame> &frameQueue, int pixelCount, int baudRate,
                                         int uart2RxPin, int uart2TxPin)
    : AbstractArtnetHandler(frameQueue, pixelCount), baudRate_(baudRate), UART2_RX_PIN_(uart2RxPin),
      UART2_TX_PIN_(uart2TxPin) {
    // Flush UART RX HW buffer to SW buffer on every received byte
    Serial2.setRxFIFOFull(1);

    Serial2.setRxBufferSize(UNIVERSE_COUNT_ * ART_DMX_MAXIMUM_LENGTH);

    onArtDmxFrame_ = [this](uint16_t universeIndex, uint16_t length, uint8_t sequence, uint8_t *data) {
        this->onDmxFrame(universeIndex, length, sequence, data);
    };
}

void ArtnetSerialHandler::start() {
    Serial2.begin(baudRate_, SERIAL_8N1, UART2_RX_PIN_, UART2_TX_PIN_);
    ESP_LOGI(TAG, "Started Artnet serial handler with baud rate %d", baudRate_);
}

void ArtnetSerialHandler::handleReceived() {
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
                ESP_LOGD(TAG, "(Re)-synchronized with byte stream");
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
                ESP_LOGW(TAG, "Received unsupported Artnet opcode %x", opcode);
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
                ESP_LOGW(TAG, "Received invalid dmxDataLength %d. Skipping this frame", dmxDataLength_);
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

void ArtnetSerialHandler::startOver(bool success) {
    if (!success) {
        isSynced_ = false;
    }
    currentBufferIndex_ = -1;
    maximumBufferIndex_ = UINT16_MAX;
    currentState_ = State::START_DETECTION;
}