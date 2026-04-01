#ifndef CALIBRATION_H
#define CALIBRATION_H

#include "../LSM6DSV80X/LSM6DSV80X.h"
#include "../Platform_Teensy/TeensyTime.hpp"

#include "IBus.hpp"
#include "SerialBus.hpp"

#include <Arduino.h>
#include <Wire.h>

#define SERIAL_TERMINAL_BAUD_RATE 115200
#define CALIBRATION_TIME_DELAY 100000
#define ACCEL_STATIONARY_EXPECTED_VALUE 1.0f
#define NUM_MEASUREMENTS 50
#define AXIS_NUM 6

class Calibration {
public:
    struct Offsets {
        float ax = 0.0f;
        float ay = 0.0f;
        float az = 0.0f;

        float gx = 0.0f;
        float gy = 0.0f;
        float gz = 0.0f;
    };

    
    Calibration(SerialBus& serial, LSM6DSV80X& imu);
    void get_offsets(Offsets& offsets);
    static void apply_offsets(const Offsets& offsets, LSM6DSV80X::IMU_Data& imu_data);

private:
    char axis_to_align[AXIS_NUM] = {'x', 'y', 'z', 'X', 'Y', 'Z'};

    LSM6DSV80X& _imu;
    SerialBus& _serial;
    LSM6DSV80X::IMU_Data imu_data;
    TeensyTime timer;

    void _align_axis_prompt(char axis);
};

#endif