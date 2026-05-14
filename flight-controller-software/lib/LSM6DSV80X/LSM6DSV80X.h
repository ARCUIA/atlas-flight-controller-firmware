#ifndef LSM6DSV80X_H
#define LSM6DSV80X_H

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include "../Core/src/IBus.hpp"
#include "../Core/src/ITimeSource.hpp"

#define GYRO_MAXVALUE_FS_RATIO  0.061037f  // 2000 / 32767, dps/count
#define ACCEL_MAXVALUE_FS_RATIO 0.000488f  // 16 / 32767, g/count

/**
 * @author Sam Manley
 * @brief Driver for LSM6DSV80X 6 Dof IMU
 */

class LSM6DSV80X {
public:
    struct IMU_Data {
        // Raw register counts
        int16_t ax_raw;
        int16_t ay_raw;
        int16_t az_raw;

        int16_t gx_raw;
        int16_t gy_raw;
        int16_t gz_raw;

        int16_t temperature_raw;

        // Processed values
        float ax_g;
        float ay_g;
        float az_g;

        float gx_dps;
        float gy_dps;
        float gz_dps;

        float temperature_c;

        // Timestamp
        uint32_t time;
    };

    LSM6DSV80X(IBus& bus, ITimeSource& time) : _bus(bus), _time(time) {}

    bool begin();
    bool raw_read(IMU_Data& data);
    bool read(IMU_Data &data);
    bool sense_event(IMU_Data &data);

    void cal_ZRL_Gyro(float gcal[], int size);

    void setupYInterrupt();

private:
    IBus& _bus;
    ITimeSource& _time;

    // ZRL values calculated by cal_ZRL_Gyro. Use in read().
    int16_t _bias_g_x = 0;
    int16_t _bias_g_y = 0;
    int16_t _bias_g_z = 0;
    
};

#endif