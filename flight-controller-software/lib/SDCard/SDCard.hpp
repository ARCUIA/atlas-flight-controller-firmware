#ifndef SDCard_HPP
#define SDCard_HPP

#include <Arduino.h>
#include <SPI.h>
#include <SdFat.h>
#include "flight_data.hpp"

#define PATH_TO_SD_DATA "/rocket"
#define SD_DATA_FILENAME "/rocket/data.txt"
#define BUFFER_SIZE 20

class SDCard {
public:
    SDCard(int ss_pin, SPIClass& spi) : ss_pin(ss_pin), spi(spi) {}

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
            buffer_count = 0;
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
            const flight_data& data = buffer[i];

            file.print(data.timestamp_us);
            file.print(",");

            file.print(data.ax_g);
            file.print(",");
            file.print(data.ay_g);
            file.print(",");
            file.print(data.az_g);
            file.print(",");

            file.print(data.gx_dps);
            file.print(",");
            file.print(data.gy_dps);
            file.print(",");
            file.print(data.gz_dps);
            file.print(",");

            file.print(data.roll);
            file.print(",");

            file.print(data.gps_fix_valid);
            file.print(",");
            file.print(data.gps_latitude_deg);
            file.print(",");
            file.print(data.gps_longitude_deg);
            file.print(",");
            file.print(data.gps_altitude_m);

            file.println();
        }

        file.close();
        buffer_count = 0;

        Serial.println("wrote data in buffer to sd card");
        return true;
    }

    bool save_to_buffer(const flight_data& data) {
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
    flight_data buffer[BUFFER_SIZE];
};

#endif