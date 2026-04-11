#include "RFD900XUS.h"

RFD900XUS::RFD900XUS(HardwareSerial& radio) : _radio(radio) {
}

void RFD900XUS::begin(uint32_t baud) {
    _radio.begin(baud);
}

bool RFD900XUS::tx_base_station(const telemetry_packet& data) {
    _radio.write((const uint8_t*)&data, sizeof(data));
    return true;
}

bool RFD900XUS::send_text(const char* text) {
    _radio.print(text);
    return true;
}


bool RFD900XUS::available() {
    return _radio.available() > 0;
}

int RFD900XUS::read() {
    return _radio.read();
}

bool RFD900XUS::receive_command(char* buffer, uint16_t buffer_size) {

    uint16_t i = 0;

    while (_radio.available() > 0 && i < buffer_size - 1) {
        char c = _radio.read();

        if (c == '\n') {
            buffer[i] = '\0';
            return true;
        }

        buffer[i] = c;
        i ++;
    }

    buffer[i] = '\0';
    return false;
}
