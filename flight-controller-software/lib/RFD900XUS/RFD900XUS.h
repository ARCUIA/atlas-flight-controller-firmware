#ifndef RFD900XUS_H
#define RFD900XUS_H

#include <stdint.h>
#include <Arduino.h>
#include "../LSM6DSV80X/LSM6DSV80X.h"

class RFD900XUS {
public:
    struct GPS_Data {
        float latitude;
        char latitude_dir;    // 'N' or 'S'
        float longitude;
        char longitude_dir;   // 'E' or 'W'
        float altitude;
        uint8_t fix_quality;  // 0=invalid, 1=GPS fix, 2=DGPS fix
    };

    struct telemetry_packet {
        LSM6DSV80X::IMU_Data imu_data;
        GPS_Data gps_data;
    };

    RFD900XUS(HardwareSerial& radio);
    void begin(uint32_t baud);

    bool tx_base_station(const telemetry_packet& data);
    bool send_text(const char* text);
    bool is_base_station_command_available();
    bool available();
    int read();
    bool receive_command(char* buffer, uint16_t buffer_size);

private:
    HardwareSerial& _radio;
};

#endif 