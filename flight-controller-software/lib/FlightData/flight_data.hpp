// One central struct that has the processed data. this will contain processed raw imu data, processed data for the radio to send, processed GPS data
// This is the source of current up to date flight data that is stored in SRAM. All peripherals operate off this struct


// This is the source of current, up-to-date flight data stored in SRAM.
// All peripherals operate off this struct.
// Units are real physical units, not raw register counts.
#include <cstdint>

#ifndef FLIGHT_DATA_HPP
#define FLIGHT_DATA_HPP

struct flight_data {
    // Timestamp
    uint32_t timestamp_us;

    float ax_g;
    float ay_g;
    float az_g;

    float gx_dps;
    float gy_dps;
    float gz_dps;

    float mag_x_ut;
    float mag_y_ut;
    float mag_z_ut;

    float roll;

    bool gps_fix_valid;
    float gps_latitude_deg;
    float gps_longitude_deg;
    float gps_altitude_m;
};


#endif