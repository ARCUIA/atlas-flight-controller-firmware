#ifndef SERIALBUS_HPP
#define SERIALBUS_HPP

#include "../Core/src/IBus.hpp"
#include <Arduino.h>
#include <SoftwareSerial.h>

class SerialBus : public IBus {
public:
    SerialBus(int rx_pin, int tx_pin, int baud_rate) : _serial(rx_pin, tx_pin), _baud_rate(baud_rate) {}

    bool write(uint8_t reg, uint8_t value) override {
        (void)reg;
        return _serial.write(value) == 1;
    }

    bool read(uint8_t reg, uint8_t* buffer, uint8_t length) override {
        (void)reg;

        if (!_serial.available()) {
            return false;
        }

        uint8_t count = 0;
        while (count < length && _serial.available()) {
            buffer[count++] = _serial.read();
        }

        return count == length;
    }

    void begin() override {
        _serial.begin(_baud_rate);
    }

    bool send_message(char* message) {
        return _serial.print(message) > 0;
    }

    bool send_bytes(uint8_t* data, size_t size) {
        return _serial.write(data, size) == size;
    }

    bool available() {
        return _serial.available();
    }

private:
    SoftwareSerial _serial;
    int _baud_rate;
};

#endif