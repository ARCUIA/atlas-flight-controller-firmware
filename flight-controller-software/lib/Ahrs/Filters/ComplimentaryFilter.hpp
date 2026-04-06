#ifndef COMPLEMENTARY_HPP
#define COMPLEMENTARY_HPP

#define RAD_TO_DEG 57.29578f

#include <cmath>
#include "../Filter.h"

class ComplementaryFilter : public Filter {

public:
    ComplementaryFilter(float a_weight, float g_weight) : Filter(), accelerometer_weight(a_weight), gyroscope_weight(g_weight) {
        last_timer_value = timer.now_us();
        first_update = true;
        // Weights must add to 1
        // In addition, assume that the data passed into this filter is already calibrated with Calibration::apply_offsets
        
    }

    bool update(Prediction& prediction, const Measurements& measurements) override {

        float accelerometer_predicted_roll = atan2(measurements.imu.ay, measurements.imu.az) * RAD_TO_DEG;
        uint32_t current_timer_value = timer.now_us();
        uint32_t time_elapsed = current_timer_value - last_timer_value;
        last_timer_value = current_timer_value;
        float gyroscope_predicted_roll = prediction.roll + (measurements.imu.gy * (time_elapsed / 1000000.0f)); 
        
        float temp = -1;
        if (first_update){
            float temp = gyroscope_weight;
            gyroscope_weight = 0;
        }

        float predicted_roll = (accelerometer_weight * accelerometer_predicted_roll) + (gyroscope_weight * gyroscope_predicted_roll);
        prediction.roll = predicted_roll;

        if (first_update){
            gyroscope_weight = temp;
            first_update = false;
        }
    
        return true;
    }

private:
    float accelerometer_weight;
    float gyroscope_weight;
    uint32_t last_timer_value;
    bool first_update;
};

#endif