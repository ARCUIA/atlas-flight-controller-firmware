#ifndef COMPLEMENTARY_HPP
#define COMPLEMENTARY_HPP

#include <cmath>
#include "../Filter.h"

class ComplementaryFilter : public Filter {
public:
    bool update(Prediction& prediction, const Measurements& measurements) override {
        prediction.roll = atan2(measurements.imu.ay, measurements.imu.az);
        return true;
    }
};

#endif