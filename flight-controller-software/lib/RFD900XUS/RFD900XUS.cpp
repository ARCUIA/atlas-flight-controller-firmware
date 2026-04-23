#include "RFD900XUS.h"

RFD900XUS::RFD900XUS(HardwareSerial& radio) : _radio(radio) {
}

void RFD900XUS::begin(uint32_t baud) {
    _radio.begin(baud);
}

// Packet,time,yaw,pitch,roll,altitude, accel x, accel y, accel z, latitude, longitude

/*
Packet,time,yaw,pitch,roll,altitude, accel x, accel y, accel z, latitude, longitude

Christian Carlos
Christian Carlos8:54 PM
Comma-separated, 
Must be all numeric values,
Must have at least 11 fields,
Each line must end with newline (\n),
First character must be a digit (or it gets ignored) (edited)
*/


bool RFD900XUS::tx_base_station(const telemetry_packet& data) {
    _radio.print(data.packet);
    _radio.print(',');

    _radio.print(data.time_ms);
    _radio.print(',');

    _radio.print(data.yaw);
    _radio.print(',');

    _radio.print(data.pitch);
    _radio.print(',');

    _radio.print(data.roll);
    _radio.print(',');

    _radio.print(data.altitude_m);
    _radio.print(',');

    _radio.print(data.imu_data.ax);
    _radio.print(',');

    _radio.print(data.imu_data.ay);
    _radio.print(',');

    _radio.print(data.imu_data.az);
    _radio.print(',');

    _radio.print(data.latitude);
    _radio.print(',');

    _radio.print(data.longitude);
    _radio.print('\n');

    return true;
}


bool RFD900XUS::send_message(const char* text) {
    _radio.println(text);
    return true;
}

int RFD900XUS::available() {
    return _radio.available();
}

int RFD900XUS::read() {
    return _radio.read();
}



/*

BS: “PING” <-> Rocket: “PONG” (edited)
BS: “ARM” <-> Rocket: “ARMED” (edited)
BS: “RESET” <-> Rocket: “RESET_OK”*

*/



bool RFD900XUS::is_command_available() {
    return _radio.available() > 0;
}


bool RFD900XUS::receive_command(std::string& buffer) { // use std::string because it makes comparing easy

    while (_radio.available() > 0) {
        char c = _radio.read();
        buffer.push_back(c);

        if (buffer == "ARM") {
            return true;
        }
        if (buffer == "PING") {
            return true;
        }
        if (buffer == "RESET") {
            return true;
        }

        if (buffer.size() >= RADIO_RECEIVE_LIMIT) {
            // Clear everything in the radio. 
            while (this->is_command_available()) {
                _radio.read();
            }
            buffer.clear();
            return false;
        }
    }

    return false;
}