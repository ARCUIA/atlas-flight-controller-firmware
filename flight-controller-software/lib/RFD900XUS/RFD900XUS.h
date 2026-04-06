#ifndef RFD900XUS_H
#define RFD900XUS_H

#include <stdint.h>
#include "../LSM6DSV80X/LSM6DSV80X.h"
#include "SerialBus.hpp"

class RFD900XUS {
public:

    struct telemetry_packet {
        LSM6DSV80X::IMU_Data imu_data;
        // Add more later if we need to transmit more
    };

    RFD900XUS(SerialBus& radio_bus);

    bool transmit_to_base_station(const telemetry_packet& data);
    bool is_base_station_command_available();

private:
    SerialBus _radio_bus;
};

#endif 