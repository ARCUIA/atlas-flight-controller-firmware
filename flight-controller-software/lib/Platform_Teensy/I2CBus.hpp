#ifndef I2CBUS_H
#define I2CBUS_H

#include <Arduino.h>
#include <Wire.h>
#include "../Core/src/IBus.hpp"

class I2CBus : public IBus {
    public :
        I2CBus(TwoWire &wire, uint8_t addr) : _wire(wire), _addr(addr) {}

        void begin()
        {
            _wire.begin();
        }

        bool write(uint8_t reg, uint8_t value) override
        {
            _wire.beginTransmission(_addr);
            _wire.write(reg);
            _wire.write(value);
            return (_wire.endTransmission() == 0);
        }

        bool read(uint8_t reg, uint8_t* buffer, uint8_t length) override
        {
            _wire.beginTransmission(_addr);
            _wire.write(reg);
            if (_wire.endTransmission(false) != 0)
                return false;

            _wire.requestFrom(_addr, length);

            for (uint8_t i = 0; i < length && _wire.available(); i++)
                buffer[i] = _wire.read();

            return true;
        }

    private : 
        TwoWire & _wire;
        uint8_t _addr;
};

#endif