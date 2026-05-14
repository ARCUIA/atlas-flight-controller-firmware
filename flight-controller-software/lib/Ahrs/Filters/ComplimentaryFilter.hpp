#ifndef COMPLEMENTARY_HPP
#define COMPLEMENTARY_HPP

#include <cmath>
#include "../Filter.h"

class ComplementaryFilter : public Filter {
public:
    ComplementaryFilter(): Filter() {
        last_timer_value = timer.now_us();
        // Weights must add to 1.
        // Assumes data passed into this filter is already processed/calibrated.
    }

    bool update(Prediction& prediction, const flight_data& data) override {

        uint32_t current_timer_value = timer.now_us();
        uint32_t time_elapsed = current_timer_value - last_timer_value;
        last_timer_value = current_timer_value;

        float dt = time_elapsed / 1000000.0f;
        float gyroscope_predicted_roll = prediction.roll + (data.gx_dps * dt);

        float predicted_roll = gyroscope_predicted_roll;
        predicted_roll = std::fmod(predicted_roll, 360.0f);
        prediction.roll = predicted_roll;

        return true;
    }

private:
    uint32_t last_timer_value;
};

#endif