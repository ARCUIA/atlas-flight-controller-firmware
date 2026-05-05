#ifndef RFD900XUS_H
#define RFD900XUS_H

#include <stdint.h>
#include <Arduino.h>
#include <cstring>
#include "../FlightData/flight_data.hpp"

class RFD900XUS {
public:
    static const uint8_t RADIO_RECEIVE_LIMIT = 32;
    static const int NUM_COMMANDS = 3;

    static constexpr char* POSSIBLE_COMMANDS[NUM_COMMANDS] = {
        "PING",
        "ARM",
        "RESET"
    };

    explicit RFD900XUS(HardwareSerial& radio);

    void begin(uint32_t baud);

    bool tx_base_station(const flight_data& data);
    bool send_message(const char* text);

    int available();
    int read();

    bool is_command_available();
    bool receive_command(char* buffer);

private:
    HardwareSerial& _radio;
    bool does_received_command_exist(const char* command);
};

#endif