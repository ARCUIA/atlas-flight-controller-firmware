#ifndef LSM6DSV80X_H
#define LSM6DSV80X_H

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include "../Core/src/IBus.hpp"
#include "../Core/src/ITimeSource.hpp"

/**
 * @author Sam Manley
 * @brief Driver for LSM6DSV80X 6 Dof IMU
 */

class LSM6DSV80X {
    public:
        struct IMU_Data {
            float ax, ay, az;   // m/s/s
            float gx, gy, gz;   // dps
            float temperature;  // C
            uint32_t time;      // uS
        };

        LSM6DSV80X(IBus& bus, ITimeSource& time) : _bus(bus), _time(time) {}

        bool begin();
        bool read(IMU_Data& data);
        void cal_ZRL_Gyro(float gcal[], int size);  // Can store the cal in the IMU itself, just returning values for now
        

    private :
        IBus& _bus;
        ITimeSource& _time;
};

#endif