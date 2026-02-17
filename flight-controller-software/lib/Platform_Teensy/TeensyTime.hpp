#ifndef TEENSYTIME_H
#define TEENSYTIME_H

#include "../Core/src/ITimeSource.hpp"
#include <Arduino.h>

class TeensyTime : public ITimeSource
{
public:
    uint32_t now_us() const override
    {
        return micros();
    }
};


#endif