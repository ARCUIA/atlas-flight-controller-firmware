#ifndef SDCard_HPP
#define SDCard_HPP

#include <Arduino.h>
#include <SPI.h>
#include <SdFat.h>
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

    SDCard(int ss_pin, SPIClass& spi)
        : ss_pin(ss_pin), spi(spi) {}

    bool begin() {
        Serial.println("SD: begin start");

        if (&spi == &SPI1) {
            SPI1.setMOSI(26);
            SPI1.setMISO(39);
            SPI1.setSCK(27);
            SPI1.setCS(ss_pin);
        }

        initialized = false;

        pinMode(ss_pin, OUTPUT);
        digitalWrite(ss_pin, HIGH);

        spi.begin();
        delay(10);

        Serial.println("SD: calling sd.begin");

        // Use conservative settings for bring-up.
        // Once stable, you can try DEDICATED_SPI and SD_SCK_MHZ(4), 8, or 16.
        SdSpiConfig config(
            ss_pin,
            SHARED_SPI,
            SD_SCK_MHZ(1),
            &spi
        );

        if (!sd.begin(config)) {
            Serial.println("SD init failed");
            sd.initErrorPrint(&Serial);
            Serial.flush();
            return false;
        }

        Serial.println("SD: init OK");

        if (!sd.exists(PATH_TO_SD_DATA)) {
            Serial.println("SD: creating /rocket");

            if (!sd.mkdir(PATH_TO_SD_DATA)) {
                Serial.println("Failed to create /rocket directory");
                Serial.flush();
                return false;
            }
        }

        initialized = true;

        Serial.println("Initialized sd card");
        Serial.flush();
        return true;
    }

    bool buffered_write() {
        if (!initialized) {
            Serial.println("SD not initialized; skipping write");
            buffer_count = 0;   // prevents repeated failed writes forever
            return false;
        }

        if (buffer_count <= 0) {
            return false;
        }

        FsFile file = sd.open(SD_DATA_FILENAME, O_WRONLY | O_CREAT | O_APPEND);

        if (!file) {
            Serial.println("Failed to open SD data file");
            sd.errorPrint(&Serial);
            Serial.flush();
            return false;
        }

        for (int i = 0; i < buffer_count; i++) {
            const SD_card_data& data = buffer[i];

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

        file.close();
        buffer_count = 0;

        Serial.println("wrote data in buffer to sd card");
        return true;
    }

    bool save_to_buffer(const SD_card_data& data) {
        if (!initialized) {
            return false;
        }

        if (buffer_count >= BUFFER_SIZE) {
            return false;
        }

        buffer[buffer_count] = data;
        buffer_count++;

        return true;
    }

    int get_buffer_count() const {
        return buffer_count;
    }

    bool is_initialized() const {
        return initialized;
    }

private:
    int ss_pin;
    SPIClass& spi;
    SdFat sd;
    bool initialized = false;

    int buffer_count = 0;
    SD_card_data buffer[BUFFER_SIZE];
};

#endif