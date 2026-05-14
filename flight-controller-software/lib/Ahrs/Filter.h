#ifndef FILTER_H
#define FILTER_H

#include <Arduino.h>
#include <Wire.h>

#include "flight_data.hpp"
#include "../Platform_Teensy/TeensyTime.hpp"

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

    virtual bool update(Prediction& prediction, const flight_data& data) = 0;

    static float compute_roll(float ay_g, float az_g) {
        return atan2(ay_g, az_g) * RAD_TO_DEG;
    }

protected:
    TeensyTime timer; // I know they say not to use protected in software design but I dont care
};

#endif