#ifndef COMPLEMENTARY_HPP
#define COMPLEMENTARY_HPP

#include <cmath>
#include "../Filter.h"

class ComplementaryFilter : public Filter {

public:
    ComplementaryFilter(float a_weight, float g_weight) : Filter(), accelerometer_weight(a_weight), gyroscope_weight(g_weight) {}

    bool update(Prediction& prediction, const Measurements& measurements) override {

        float accelerometer_predicted_roll = atan2(measurements.imu.ay, measurements.imu.az);

        uint32_t current_timer_value = timer.now_us();
        uint32_t time_elapsed = current_timer_value - last_timer_value;
        last_timer_value = current_timer_value;
        float gyroscope_predicted_roll = prediction.roll + (measurements.imu.gy * (time_elapsed / 1000000.0f)); 
        float predicted_roll = (accelerometer_weight * accelerometer_predicted_roll) + (gyroscope_weight * gyroscope_predicted_roll);
        prediction.roll = predicted_roll;
    
        return true;
    }

private:
    float accelerometer_weight;
    float gyroscope_weight;
    uint32_t last_timer_value = timer.now_us();
};

#endif