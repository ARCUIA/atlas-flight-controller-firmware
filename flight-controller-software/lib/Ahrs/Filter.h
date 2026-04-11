#ifndef FILTER_H
#define FILTER_H

#include "../LSM6DSV80X/LSM6DSV80X.h"
#include "../Platform_Teensy/TeensyTime.hpp"

#include "IBus.hpp"

#include <Arduino.h>
#include <Wire.h>

#define RAD_TO_DEG 57.29578f

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

    float compute_roll(float ay, float az){
        return atan2(ay, az) * RAD_TO_DEG;
    }

    protected:
        TeensyTime timer; // I know they say not to use protected in software design but I dont care


    

};

#endif