#ifndef FILTER_H
#define FILTER_H

#include "../LSM6DSV80X/LSM6DSV80X.h"
#include "../Platform_Teensy/TeensyTime.hpp"

#include "IBus.hpp"
#include "SerialBus.hpp"

#include <Arduino.h>
#include <Wire.h>

class Filter {
public:

    virtual ~Filter() = default;

    struct Prediction {
        float roll;
        // Maybe add these in the future?
        // float pitch;
        // float yaw;
      
    };

    struct Measurements {
        LSM6DSV80X::IMU_Data imu;

        float mx, my, mz;     

        // Maybe this in the future?
        /*
        float b_pressure;        
        float b_temperature;       

        float gps_latitude;        
        float gps_longitude;
        float gps_altitude;
        float gps_speed;
        */
    };

    virtual bool update(Prediction& prediction, const Measurements& measurements) = 0;

    protected:
        TeensyTime timer; // I know they say not to use protected in software design but I dont care
};

#endif