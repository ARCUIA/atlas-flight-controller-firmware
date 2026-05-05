#ifndef COMPLEMENTARY_HPP
#define COMPLEMENTARY_HPP

#include <cmath>
#include "../Filter.h"

class ComplementaryFilter : public Filter {
public:
    ComplementaryFilter(float a_weight, float g_weight)
        : Filter(),
          accelerometer_weight(a_weight),
          gyroscope_weight(g_weight) {
        last_timer_value = timer.now_us();
        first_update = true;

        // Weights must add to 1.
        // Assumes data passed into this filter is already processed/calibrated.
    }

    bool update(Prediction& prediction, const flight_data& data) override {
        float accelerometer_predicted_roll =
            Filter::compute_roll(data.ay_g, data.az_g);

        accelerometer_predicted_roll =
            std::fmod(accelerometer_predicted_roll, 360.0f);

        uint32_t current_timer_value = timer.now_us();
        uint32_t time_elapsed = current_timer_value - last_timer_value;
        last_timer_value = current_timer_value;

        float dt = time_elapsed / 1000000.0f;

        float gyroscope_predicted_roll =
            prediction.roll + (data.gx_dps * dt);

        if (first_update) {
            prediction.roll = accelerometer_predicted_roll;
            first_update = false;
            return true;
        }

        float predicted_roll =
            (accelerometer_weight * accelerometer_predicted_roll) +
            (gyroscope_weight * gyroscope_predicted_roll);

        prediction.roll = predicted_roll;

        return true;
    }

private:
    float accelerometer_weight;
    float gyroscope_weight;
    uint32_t last_timer_value;
    bool first_update;
};

#endif