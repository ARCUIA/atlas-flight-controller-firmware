#ifndef RFD900XUS_H
#define RFD900XUS_H

#include <stdint.h>
#include <Arduino.h>
#include "../LSM6DSV80X/LSM6DSV80X.h"

class RFD900XUS {
public:
    struct telemetry_packet {
        LSM6DSV80X::IMU_Data imu_data;
        // Add more later if we need to transmit more
    };

    RFD900XUS(HardwareSerial& radio);
    void begin(uint32_t baud);

    bool tx_base_station(const telemetry_packet& data);
    bool send_text(const char* text);
    bool send_textln(const char* text);
    bool is_base_station_command_available();
    bool available();
    int read();

private:
    HardwareSerial& _radio;
};

#endif 