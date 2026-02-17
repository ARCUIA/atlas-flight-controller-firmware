#ifndef SPIBUS_H
#define SPIBUS_H

#include <Arduino.h>
#include <SPI.h>
#include "../Core/src/IBus.hpp"

class SPIBus : public IBus {
    public:
        SPIBus(SPIClass& spi, uint8_t cs) : _spi(spi), _cs(cs) {}

        void begin() 
        {
            pinMode(_cs, OUTPUT);
            digitalWrite(_cs, HIGH);
            _spi.begin();
        }
        
        bool write(uint8_t reg, uint8_t value) override
        {
            digitalWrite(_cs, LOW);
            _spi.transfer(reg & 0x7F);
            _spi.transfer(value);
            digitalWrite(_cs, HIGH);
            return true;
        }

        bool read(uint8_t reg, uint8_t* buffer, uint8_t length) override
        {
            digitalWrite(_cs, LOW);
            _spi.transfer(reg | 0x80);
            for (uint8_t i = 0; i < length; i++)
                buffer[i] = _spi.transfer(0x00);
            digitalWrite(_cs, HIGH);
            return true;
        }


    private:
        SPIClass& _spi;
        uint8_t _cs;
};

#endif