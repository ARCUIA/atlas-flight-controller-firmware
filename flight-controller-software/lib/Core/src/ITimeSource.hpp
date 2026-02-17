#ifndef ITIMESOURCE_H
#define ITIMESOURCE_H

#include <stdint.h>

class ITimeSource {
    public:
        virtual ~ITimeSource() = default;
        virtual uint32_t now_us() const = 0;
};

#endif