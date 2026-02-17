#ifndef IBUS_H
#define IBUS_H

#include <stdint.h>

class IBus {
    public:
        virtual ~IBus() = default;

        virtual bool write(uint8_t reg, uint8_t value) = 0;
        virtual bool read(uint8_t reg, uint8_t* buffer, uint8_t length) = 0;
        virtual void begin() = 0;
};

#endif