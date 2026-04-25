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


bool RFD900XUS::receive_command(char* buffer) { // use std::string because it makes comparing easy

    uint16_t buffer_index = 0;
    buffer[0] = '\0'; // Reset the buffer

    

    while (_radio.available() > 0) {
        char c = _radio.read();
        buffer[buffer_index] = c;
        buffer_index ++;
        buffer[buffer_index] = '\0';


       // buffer.push_back(c);

        if (this->does_received_command_exist(buffer)){
            return true;
        }
       

        if (buffer_index >= RADIO_RECEIVE_LIMIT) {
            // Clear everything
            while (this->is_command_available()) {
                _radio.read();
            }
            buffer[0] = '\0'; 
            return false;
        }
    }

    return false;

}

bool RFD900XUS::does_received_command_exist(const char* received_command){
    for (int i = 0; i < NUM_COMMANDS; i++) {
        if (strcmp(received_command, POSSIBLE_COMMANDS[i]) == 0) {
            return true;
        }
    }

    return false;
}

