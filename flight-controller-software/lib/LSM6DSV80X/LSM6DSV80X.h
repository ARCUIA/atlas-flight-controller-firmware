#ifndef LSM6DSV80X_H
#define LSM6DSV80X_H

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>

/**
 * @author Sam Manley
 * @brief Driver for LSM6DSV80X 6 Dof IMU
 */

class LSM6DSV80X {
    public:
        struct IMU_Data {
            float ax, ay, az;   // m/s/s
            float gx, gy, gz;   // dps
            float temperature;  // C
        };

        LSM6DSV80X(SPIClass& spi, uint8_t csPin);

        bool begin();
        bool drdy();
        bool read(IMU_Data &data);

    private : 
        SPIClass &_spi;
        uint8_t _cs;

        void write_register(uint8_t reg, uint8_t value);
        uint8_t read_register(uint8_t reg);
        void read_registers(uint8_t reg, uint8_t *buffer, uint8_t length);
};

#endif