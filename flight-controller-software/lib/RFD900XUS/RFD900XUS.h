#ifndef RFD900XUS_H
#define RFD900XUS_H

#include <stdint.h>
#include <string>
#include <Arduino.h>
#include "../LSM6DSV80X/LSM6DSV80X.h"

class RFD900XUS {
public:
    static constexpr uint8_t RADIO_RECEIVE_LIMIT = 5;
    static constexpr int NUM_COMMANDS = 3;
    static constexpr char* POSSIBLE_COMMANDS[NUM_COMMANDS]  = {"PING", "ARM", "RESET"};

    struct telemetry_packet {
        uint32_t packet; // What is packet?
        uint32_t time_ms;

        float yaw;
        float pitch; // We only getting roll info for now
        float roll;

        float altitude_m; // Are we actually using barometer?

        LSM6DSV80X::IMU_Data imu_data;

        float latitude;
        float longitude;
    };

    explicit RFD900XUS(HardwareSerial& radio);

    void begin(uint32_t baud);

    bool tx_base_station(const telemetry_packet& data);
    bool send_message(const char* text);

    int available();

    bool is_command_available();
    int read();

    bool receive_command(char* buffer);

private:
    HardwareSerial& _radio;
    bool does_received_command_exist(const char* command);
};

#endif