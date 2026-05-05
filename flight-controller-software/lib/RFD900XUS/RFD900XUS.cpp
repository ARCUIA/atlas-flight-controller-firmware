#include "RFD900XUS.h"

RFD900XUS::RFD900XUS(HardwareSerial& radio) : _radio(radio) {
}

void RFD900XUS::begin(uint32_t baud) {
    _radio.begin(baud);
}

// Base station format:
// Packet,time,yaw,pitch,roll,altitude,accel_x,accel_y,accel_z,latitude,longitude
//
// Requirements:
// - Comma-separated
// - All numeric values
// - At least 11 fields
// - Each line ends with newline '\n'
// - First character must be a digit

bool RFD900XUS::tx_base_station(const flight_data& data) {
    static uint32_t packet_count = 0;

    // Packet
    _radio.print(packet_count++);
    _radio.print(',');

    // Time in milliseconds
    _radio.print(data.timestamp_us / 1000);
    _radio.print(',');

    // Yaw placeholder
    _radio.print(0);
    _radio.print(',');

    // Pitch placeholder
    _radio.print(0);
    _radio.print(',');

    // Roll
    _radio.print(data.roll);
    _radio.print(',');

    // Altitude
    _radio.print(data.gps_altitude_m);
    _radio.print(',');

    // Acceleration X, Y, Z
    _radio.print(data.ax_g);
    _radio.print(',');

    _radio.print(data.ay_g);
    _radio.print(',');

    _radio.print(data.az_g);
    _radio.print(',');

    // Latitude
    _radio.print(data.gps_latitude_deg, 6);
    _radio.print(',');

    // Longitude
    _radio.print(data.gps_longitude_deg, 6);

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
BS: “PING”  <-> Rocket: “PONG”
BS: “ARM”   <-> Rocket: “ARMED”
BS: “RESET” <-> Rocket: “RESET_OK”
*/

bool RFD900XUS::is_command_available() {
    return _radio.available() > 0;
}

bool RFD900XUS::receive_command(char* buffer) {
    uint16_t buffer_index = 0;
    buffer[0] = '\0';

    while (_radio.available() > 0) {
        char c = _radio.read();

        buffer[buffer_index] = c;
        buffer_index++;
        buffer[buffer_index] = '\0';

        if (this->does_received_command_exist(buffer)) {
            return true;
        }

        if (buffer_index >= RADIO_RECEIVE_LIMIT) {
            while (this->is_command_available()) {
                _radio.read();
            }

            buffer[0] = '\0';
            return false;
        }
    }

    return false;
}

bool RFD900XUS::does_received_command_exist(const char* received_command) {
    for (int i = 0; i < NUM_COMMANDS; i++) {
        if (strcmp(received_command, POSSIBLE_COMMANDS[i]) == 0) {
            return true;
        }
    }

    return false;
}