#ifndef SDCard_HPP
#define SDCard_HPP

#include <Arduino.h>
#include <SD.h>
#include "../LSM6DSV80X/LSM6DSV80X.h"

#define PATH_TO_SD_DATA "/rocket"
#define SD_DATA_FILENAME "/rocket/data.txt"
#define BUFFER_SIZE 20

class SDCard {
public:
    struct SD_card_data {
        LSM6DSV80X::IMU_Data imu_data;

        float gps_latitude = 0.0f;
        float gps_longitude = 0.0f;
        float gps_altitude = 0.0f;

        float mag_x = 0.0f;
        float mag_y = 0.0f;
        float mag_z = 0.0f;

        uint32_t timestamp_us = 0;
    };

    SDCard(int ss_pin) : ss_pin(ss_pin) {}

    bool begin() {
        if (!SD.begin(ss_pin)) {
            return false;
        }

        if (!SD.exists(PATH_TO_SD_DATA)) {
            SD.mkdir(PATH_TO_SD_DATA);
        }

        Serial.println("Initialized sd card");
        return true;
    }

    bool buffered_write() {
        if (buffer_count <= 0) {
            return false;
        }

        File file = SD.open(SD_DATA_FILENAME, FILE_WRITE);
        if (!file) {
            return false;
        }

        for (int i = 0; i < buffer_count; i++) {
            SD_card_data data = buffer[i];

            file.print(data.timestamp_us);
            file.print(",");
            file.print(data.imu_data.ax);
            file.print(",");
            file.print(data.imu_data.ay);
            file.print(",");
            file.print(data.imu_data.az);
            file.print(",");
            file.print(data.imu_data.gx);
            file.print(",");
            file.print(data.imu_data.gy);
            file.print(",");
            file.print(data.imu_data.gz);
            file.print(",");
            file.print(data.gps_latitude);
            file.print(",");
            file.print(data.gps_longitude);
            file.print(",");
            file.print(data.gps_altitude);
            file.print(",");
            file.print(data.mag_x);
            file.print(",");
            file.print(data.mag_y);
            file.print(",");
            file.print(data.mag_z);
            file.println();
        }

        Serial.println("wrote data in buffer to sd card");

        buffer_count = 0;
        file.close();
        return true;
    }

    bool save_to_buffer(const SD_card_data& data) {
        if (buffer_count >= buffer_size) {
            return false;
        }

        buffer[buffer_count] = data;
        buffer_count++;

        Serial.println("wrote data to sd card buffer");
        return true;
    }

    int get_buffer_count() {
        return buffer_count;
    }

private:
    int ss_pin;
    int buffer_count = 0;
    int buffer_size = BUFFER_SIZE;
    SD_card_data buffer[BUFFER_SIZE];
};

#endif