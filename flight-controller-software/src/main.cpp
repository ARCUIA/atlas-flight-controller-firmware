/**
 * @author Sam Manley
 * @brief Flight Controller Software for ARCUIA 2026 Secret Message Rocket
 * @date 2-15-26
 * @version 1.0.0
 */

// TODO: Kalman filter?
// TODO: Pitch and Yaw?
// TODO: Actually test this



// Libraries
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <FS.h>
#include <Adafruit_LIS2MDL.h>
#include <Adafruit_GPS.h>


// Header Files
#include "startup.hpp"
#include "myenums.hpp"
#include "sd_handler.hpp"
#include "../lib/LSM6DSV80X/LSM6DSV80X.h"
#include "../lib/RFD900XUS/RFD900XUS.h"
#include "../lib/Platform_Teensy/TeensyTime.hpp"
#include "../lib/Platform_Teensy/I2CBus.hpp"
#include "../lib/Platform_Teensy/SPIBus.hpp"
#include "../lib/Ahrs/Calibration.h"
#include "../lib/Ahrs/Filters/ComplimentaryFilter.hpp"
#include "../lib/Ahrs/Filter.h"
#include "../lib/SDCard/SDCard.hpp"

const int DEBUG_MODE = true; // Enable serial connection to computer for debugging
const int DEBUG_SERIAL_BAUD_RATE = 9600; // Baud rate for debugging

// Pins
const int ssPin = 38;

const int GPS_BAUD_RATE = 9600;
const int RADIO_BAUD_RATE = 115200;

// Declaring Constants Here
const uint32_t DELAY = 100000UL; // uS

// Declaring Variables Here
uint32_t now = micros();

// Periods of when to do each action
const uint32_t FILTER_PERIOD_US  = DELAY; // Do it every loop
const uint32_t GPS_PERIOD_US     = 1000000UL; // Placeholders
const uint32_t MAG_PERIOD_US     = 50000UL;
const uint32_t BARO_PERIOD_US    = 50000UL;
const uint32_t PID_PERIOD_US     = DELAY;
const uint32_t RADIO_PERIOD_US   = 100000UL;
const uint32_t SD_CARD_PERIOD_US = 100000UL;

// Starting times to decide when to do actions
uint32_t time_filter_prev = now;
uint32_t time_gps_prev = now;
uint32_t time_mag_prev = now;
uint32_t time_baro_prev = now;
uint32_t time_pid_prev = now;
uint32_t time_radio_prev = now;
uint32_t time_sd_prev = now;

const float FILTER_GYRO_WEIGHT = 0.5f;
const float FILTER_ACCEL_WEIGHT = 0.5f; // Must add to 1.0

uint32_t prev = now;
float mag_dec = 0.0f;

TeensyTime imu_time;

// Create Objects Here
SPIBus imu_bus(SPI, 10);
RFD900XUS radio(Serial5);
LSM6DSV80X imu(imu_bus, imu_time);
Adafruit_GPS gps(&Serial1);
SDCard sd_card(ssPin, SPI1);

ComplementaryFilter filter(FILTER_GYRO_WEIGHT, FILTER_ACCEL_WEIGHT);
Filter::Prediction prediction;
Filter::Measurements measurements;

Adafruit_LIS2MDL mag;

//I2CBus magnetometer;
//I2CBus barometer;

LSM6DSV80X::IMU_Data imu_data;
RFD900XUS::telemetry_packet telemetry;
Calibration::Offsets offset;

flightState state;
controlType controls = controlType::CANARDS;

void setup()
{

  if (DEBUG_MODE == true){
    Serial.begin(DEBUG_SERIAL_BAUD_RATE);
    delay(1000);
    Serial.print("INIT ROCKET");
  }

  // Keep Tests out of setup() in case of temp black/brownout.
  prediction.roll = 0.0f; // Just so we dont grab a garbage value. There is code to ignore the first value of prediction.roll so this should be overwritten after we call filter.update

  mag_dec = get_mag_dec(launchSite::IOWA_CITY);


  // Com Busses
  Wire.begin();
  radio.begin(RADIO_BAUD_RATE); // Radio
  gps.begin(GPS_BAUD_RATE); // GPS
  imu.begin(); // IMU
  sd_card.begin();

  //  magnetometer.begin() // Magnetometer
  //  barometer.begin() // Barometer
  Serial.print("INIT FINISHED");
  //Calibration calibration(radio, imu);
  //calibration.get_offsets(offset); // Runs the calibration routine. We probably want to move this somewhere else

  
  // Enable RMC and GGA
  gps.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // Set the default update rate of 1HZ
  gps.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  Serial.print("GPS COMMAND FINISHED");

}

void loop() {
  now = micros();

  if (now - prev < DELAY) {
    return;
  }
  prev = now;

  state = flightState::POWERED_ASCENT;

  char command_buffer[32] = {0};


  if (radio.is_command_available()) {
    if (radio.receive_command(command_buffer)) {

      if (strcmp(command_buffer, "ARM") == 0) {
        state = flightState::POWERED_ASCENT;
       // Calibration::apply_offsets(offset, imu_data); 
        radio.send_message("ARMED\n");
      }
      else if (strcmp(command_buffer, "PING") == 0) {
        Serial.println("RECIEVED PING");
        radio.send_message("PONG\n");
      }
      else if (strcmp(command_buffer, "RESET") == 0) {
        radio.send_message("RESET_OK\n");
      }

      command_buffer[0] = '\0';
    }
  }

  switch (state) {  
    case flightState::PREFLIGHT_IDLE:
      break;

    case flightState::POWERED_ASCENT: {
      imu.read(imu_data);

      telemetry.imu_data = imu_data;
      measurements.imu = imu_data;
/*
      if (DEBUG_MODE == true) {
        Serial.print("IMU accel: ");
        Serial.print(imu_data.ax);
        Serial.print(", ");
        Serial.print(imu_data.ay);
        Serial.print(", ");
        Serial.println(imu_data.az);

        Serial.print(" | gyro: ");
        Serial.print(imu_data.gx);
        Serial.print(", ");
        Serial.print(imu_data.gy);
        Serial.print(", ");
        Serial.println(imu_data.gz);
      }
*/

      SDCard::SD_card_data sd_data;
      sd_data.timestamp_us = now;
      sd_data.imu_data = imu_data;
      sd_data.gps_latitude = telemetry.latitude;
      sd_data.gps_longitude = telemetry.longitude;
      sd_data.gps_altitude = telemetry.altitude_m;
      sd_card.save_to_buffer(sd_data);

      if (sd_card.get_buffer_count() >= BUFFER_SIZE) {
        sd_card.buffered_write();
      }

      if (now - time_mag_prev >= MAG_PERIOD_US) {
        time_mag_prev = now;
        // mag.read
      }

      if (now - time_baro_prev >= BARO_PERIOD_US) {
        time_baro_prev = now;
        // baro.read()
      }

      if (now - time_gps_prev >= GPS_PERIOD_US) {
        time_gps_prev = now;

        if (gps.newNMEAreceived()) {
          if (gps.parse(gps.lastNMEA())) {
            telemetry.latitude = gps.latitude;
            telemetry.longitude = gps.longitude;
            telemetry.altitude_m = gps.altitude;
          }
        }
      }

      if (now - time_pid_prev >= PID_PERIOD_US) {
        time_pid_prev = now;
        // PID.control(prediction.roll)
      }

      if (now - time_radio_prev >= RADIO_PERIOD_US) {
        time_radio_prev = now;
        radio.tx_base_station(telemetry);
      }

      if (now - time_filter_prev >= FILTER_PERIOD_US) {
        time_filter_prev = now;
        filter.update(prediction, measurements);
      }

      break;
    }

    case flightState::UNPOWERED_ASCENT:
      break;

    case flightState::CHUTE_DESCENT:
      break;

    case flightState::POSTFLIGHT_IDLE:
      break;
  }
}
/*================ EXAMPLE ZONE =======================

I2CBus bus(Wire, 0x6A);
TeensyTime time;
LSM6DSV80X imu(bus, time);

SPIBus bus(SPI, 10);
TeensyTime time;
LSM6DSV80X imu(bus, time);


================ EXAMPLE ZONE =========================*/