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

bool RFD900XUS::send_textln(const char* text) {
    _radio.println(text);
    return true;
}

bool RFD900XUS::is_base_station_command_available() {
    return _radio.available() > 0; // Temporary, add more to this later
}

bool RFD900XUS::available() {
    return _radio.available() > 0;
}

int RFD900XUS::read() {
    return _radio.read();
}

// TODO: Add code here to handle base station commands
