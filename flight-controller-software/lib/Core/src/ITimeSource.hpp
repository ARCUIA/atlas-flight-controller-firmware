#ifndef ITIMESOURCE_H
#define ITIMESOURCE_H

#include <stdint.h>

class ITimeSource {
    public:
        virtual ~ITimeSource() = default;
        virtual uint32_t now_us() const = 0;
        virtual void delay_us(uint32_t time) = 0;  // Blocking
};

#endif