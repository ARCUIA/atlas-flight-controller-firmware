/**
 * @author Sam Manley
 * @brief Flight Controller Software for ARCUIA 2026 Secret Message Rocket
 * @date 2-15-26
 * @version 1.0.0
 */

// TODO: Add driver / bus for SD card and add code to store data in SD card
// TODO: Kalman filter?
// TODO: Pitch and Yaw?
// TODO: Actually test this
// TODO: Drivers for barometer and magnetometer?


// Libraries
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
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
#include "../lib/Ahrs/Calibration.h"
#include "../lib/Ahrs/Filters/ComplimentaryFilter.hpp"
#include "../lib/Ahrs/Filter.h"

// Pins
const int ssPin = 10;

const int RADIO_TX_PIN = 1; // Serial1 TX
const int RADIO_RX_PIN = 0; // Serial1 RX

const int GPS_TX_PIN = 8; // Serial2 TX
const int GPS_RX_PIN = 7; // Serial2 RX

const int GPS_BAUD_RATE = 9600;
const int RADIO_BAUD_RATE = 57600;

// Declaring Constants Here
const uint32_t DELAY = 10000UL; // uS

// Declaring Variables Here
uint32_t now = micros();

// Periods of when to do each action
const uint32_t FILTER_PERIOD_US = -1;
const uint32_t GPS_PERIOD_US = -1;
const uint32_t MAG_PERIOD_US = -1; // TODO: Put the actual periods
const uint32_t BARO_PERIOD_US = -1;
const uint32_t PID_PERIOD_US = -1;
const uint32_t RADIO_PERIOD_US = -1;
const uint32_t SD_CARD_PERIOD_US = -1;

// Starting times to decide when to do actions
uint32_t time_filter_prev = now;
uint32_t time_gps_prev = now;
uint32_t time_mag_prev = now;
uint32_t time_baro_prev = now;
uint32_t time_pid_prev = now;
uint32_t time_radio_prev = now;
uint32_t time_sd_prev = now;

uint32_t prev = now;
float mag_dec = 0.0f;

// Create Objects Here
I2CBus imu_bus(Wire,0x6A);
RFD900XUS radio(Serial1);
SerialBus gps_bus(Serial2, GPS_BAUD_RATE);

TeensyTime imu_time;
Adafruit_LIS2MDL mag;

//I2CBus magnetometer;
//I2CBus barometer;

LSM6DSV80X::IMU_Data imu_data;
RFD900XUS::telemetry_packet telemetry;
Calibration::Offsets offset;

LSM6DSV80X imu(imu_bus, imu_time);

Adafruit_GPS gps(&Serial1);

float g_weight = 0.5f;
float a_weight = 0.5f;
ComplementaryFilter filter(g_weight, a_weight);
Filter::Prediction prediction;
Filter::Measurements measurements;

flightState state;
controlType controls = controlType::CANARDS;

void setup()
{
  // Keep Tests out of setup() in case of temp black/brownout.
  prediction.roll = 0.0f; // Just so we dont grab a garbage value. There is code to ignore the first value of prediction.roll so this should be overwritten after we call filter.update

  mag_dec = get_mag_dec(launchSite::IOWA_CITY);
  
  // Com Busses
  Wire.begin();
  radio.begin(RADIO_BAUD_RATE); // Radio
  gps_bus.begin(); // GPS
  imu.begin(); // IMU

  Calibration calibration(radio, imu);
  calibration.get_offsets(offset);

  gps.begin(GPS_BAUD_RATE);
  // 
  // Enable RMC and GGA
  gps.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // Set the default update rate of 1HZ
  gps.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  GPS.LOCUS_StartLogger() // Start logging gps data 
//  magnetometer.begin() // Magnetometer
//  barometer.begin() // Barometer

  initSD(ssPin); // Returns Error if Failed.      

}

void loop() {
  now = micros();

  if (now - prev >= DELAY) {

    // Just here to test, remove later
    imu.read(imu_data); // imu_data will naturally be updated with the readings from the last imu measurement taken
    Calibration::apply_offsets(offset, imu_data); // Apply the offsets in software not hardware
    telemetry.imu_data = imu_data; // for radio
    measurements.imu = imu_data; // for filter
    
    switch (state) {  
      case flightState::PREFLIGHT_IDLE:
        break;
      
      case flightState::POWERED_ASCENT:
        // Get I2C Data
        if (now - time_mag_prev >= MAG_PERIOD_US) {
            time_mag_prev = micros();
        }

        if (now - time_baro_prev >= BARO_PERIOD_US) {
            time_baro_prev = micros();
            
            // baro.read()
        }

        // ===============
        //  TODO: Add in Drivers for sensors (LSM6DSV80X, LPS22HH)
        // ===============


        if (gps_bus.available()) {
            // read GPS bytes 
            // Issue an interrupt ?
        }

        // State Estimation


        // Control Systems

        if (now - time_pid_prev >= PID_PERIOD_US) {
          time_pid_prev = micros();
          // PID.control(prediction.roll) <- maybe like this in the future?
        }


        // Data Transmission
        if (now - time_radio_prev >= RADIO_PERIOD_US) {
          time_radio_prev = micros();
          radio.tx_base_station(telemetry);
        }

        if (now - time_filter_prev >= FILTER_PERIOD_US) {
          time_filter_prev = micros();
          filter.update(prediction, measurements);
        // filter.predict/update
        }


        break;
      case flightState::UNPOWERED_ASCENT:
        // Implement later
        break;
      case flightState::CHUTE_DESCENT:
        // Turn off Cameras
        // Turn off Motors
        break;
      case flightState::POSTFLIGHT_IDLE:
        // If not already off, turn off everything besides radio and gps
        // Radio ping GPS every 1 minute, otherwise idle.
        break;
      }
    prev = now;
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