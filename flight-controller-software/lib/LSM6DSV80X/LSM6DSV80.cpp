#include "LSM6DSV80X.h"

#define WHOAMI_REG   0x0F
#define OUT_TEMP_L   0x20  // start of 14 byte output register sequence

bool LSM6DSV80X::begin() {
    
    _bus.begin(); // I2C or SPI depending on implementation

    // need to configure default ranges and odr

}

// generic read, pass by ref IMU_Data ref for 3a,3g,1temp
bool LSM6DSV80X::read(IMU_Data& data) {

    const uint8_t BUF_LEN = 14;
    uint8_t buf[BUF_LEN] = {};

    // Get Timestamp
    uint32_t time = _time.now_us();

    // Read Data to Buffer
    _bus.read(OUT_TEMP_L, buf, BUF_LEN);

    // Assign data to struct
    data.temperature = (buf[1] << 8)  | buf[0];
    data.gx          = (buf[3] << 8)  | buf[2];
    data.gy          = (buf[5] << 8)  | buf[4];
    data.gz          = (buf[7] << 8)  | buf[6];
    data.ax          = (buf[9] << 8)  | buf[8];
    data.ay          = (buf[11] << 8) | buf[10];
    data.az          = (buf[13] << 8) | buf[12];
    data.time        = time;
}