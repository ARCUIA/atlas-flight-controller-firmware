#include "LSM6DSV80X.h"

#define WHOAMI_REG = 0x0F;

LSM6DSV80X::LSM6DSV80X(SPIClass& spi, uint8_t csPin) : _spi(spi), _cs(csPin) {}