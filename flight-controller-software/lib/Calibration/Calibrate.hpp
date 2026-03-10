#ifndef Calibrate_H
#define Calibrate_H

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

class Calibrate {
public:
    const char axis_to_align[AXIS_NUM] = {'x', 'y', 'z', 'X', 'Y', 'Z'};

    Calibrate(SerialBus& serial, LSM6DSV80X& imu) : _imu(imu), _serial(serial) {
        Serial.begin(SERIAL_TERMINAL_BAUD_RATE);
    }

    void get_offsets(float accel_offsets[3], float gyro_offsets[3]) {
        TeensyTime time;

        float accel_x = 0.0f;
        float accel_y = 0.0f;
        float accel_z = 0.0f;

        float gyro_x = 0.0f;
        float gyro_y = 0.0f;
        float gyro_z = 0.0f;

        int count_x = 0;
        int count_y = 0;
        int count_z = 0;

        for (int i = 0; i < AXIS_NUM; i++) {
            char current_axis = axis_to_align[i];
            _align_axis_prompt(current_axis);

            for (int j = 0; j < NUM_MEASUREMENTS; j++) {
                _imu.read(imu_data);

                switch (_to_lowercase(current_axis)) {
                    case 'x':
                        accel_x += (imu_data.ax - ACCEL_STATIONARY_EXPECTED_VALUE);
                        gyro_x += imu_data.gx;
                        count_x++;
                        break;

                    case 'y':
                        accel_y += (imu_data.ay - ACCEL_STATIONARY_EXPECTED_VALUE);
                        gyro_y += imu_data.gy;
                        count_y++;
                        break;

                    case 'z':
                        accel_z += (imu_data.az - ACCEL_STATIONARY_EXPECTED_VALUE);
                        gyro_z += imu_data.gz;
                        count_z++;
                        break;
                }

                time.delay_us(CALIBRATION_TIME_DELAY);
            }
        }

        accel_offsets[0] = accel_x / count_x;
        accel_offsets[1] = accel_y / count_y;
        accel_offsets[2] = accel_z / count_z;

        gyro_offsets[0] = gyro_x / count_x;
        gyro_offsets[1] = gyro_y / count_y;
        gyro_offsets[2] = gyro_z / count_z;
    }

private:
    LSM6DSV80X& _imu;
    SerialBus& _serial;
    LSM6DSV80X::IMU_Data imu_data;

    void _align_axis_prompt(char axis) {
        _serial.send_message((char*)"Align the IMU along its ");

        switch (axis) {
            case 'x':
                _serial.send_message((char*)"+X");
                break;
            case 'y':
                _serial.send_message((char*)"+Y");
                break;
            case 'z':
                _serial.send_message((char*)"+Z");
                break;
            case 'X':
                _serial.send_message((char*)"-X");
                break;
            case 'Y':
                _serial.send_message((char*)"-Y");
                break;
            case 'Z':
                _serial.send_message((char*)"-Z");
                break;
        }

        _serial.send_message((char*)" axis, then press any key.\n");

        TeensyTime time;

        while (!_serial.available()) {
            _serial.send_bytes((uint8_t*)&imu_data, sizeof(imu_data));
            time.delay_us(CALIBRATION_TIME_DELAY);
        }

        uint8_t buffer;
        while (_serial.available()) {
            _serial.read(0, &buffer, 1);
        }
    }

    char _to_lowercase(char c) {
        if (c >= 'A' && c <= 'Z') {
            return c - 'A' + 'a';
        }
        return c;
    }
};

#endif