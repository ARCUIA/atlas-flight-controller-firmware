#ifndef LIS2MDL_H
#define LIS2MDL_H

#include "Arduino.h"
#include <Adafruit_LIS2MDL.h>
#include <Adafruit_Sensor.h>
#include <SPI.h>
#include "../Core/src/IBus.hpp"
#include "../Core/src/ITimeSource.hpp"

// Dont need the id but why not
Adafruit_LIS2MDL lis2mdl = Adafruit_LIS2MDL(12345);
#define LIS2MDL_CLK 13
#define LIS2MDL_MISO 12
#define LIS2MDL_MOSI 11
#define LIS2MDL_CS 29

const int CS_MAG  = 29;

static SPISettings MAG_SPI(8000000, MSBFIRST, SPI_MODE0); // back off from the 10MHz edge

uint8_t magRead(uint8_t reg) {
    SPI.beginTransaction(MAG_SPI);
    digitalWrite(CS_MAG, LOW);
    SPI.transfer(reg | 0x80);
    uint8_t val = SPI.transfer(0x00);
    digitalWrite(CS_MAG, HIGH);
    SPI.endTransaction();
    return val;
}

void magWrite(uint8_t reg, uint8_t val) {
    SPI.beginTransaction(MAG_SPI);
    digitalWrite(CS_MAG, LOW);
    SPI.transfer(reg & 0x7F);
    SPI.transfer(val);
    digitalWrite(CS_MAG, HIGH);
    SPI.endTransaction();
}

class LIS2MDL : public Adafruit_LIS2MDL {
public:
    // Constructor
    LIS2MDL(IBus& bus, ITimeSource& time) : _bus(bus), _time(time) {}

    // ONLY WORKS ON SPI0 RIGHT NOW (hardcoded)
    bool enable4WireSPI() {
        // Change post launch but for now we can only ever use SPI0 anyways so hard coding instead of updating IBUS.
        SPI.begin();
        uint8_t reg_val = magRead(0x62);
        reg_val |= 0x04;
        magWrite(0x62, reg_val);

        // Verification read
        return (magRead(0x62) & (uint8_t)0x04);
    }

    // Add these
    read()
    write() 

private:
    IBus& _bus;
    ITimeSource& _time;

}



#endif