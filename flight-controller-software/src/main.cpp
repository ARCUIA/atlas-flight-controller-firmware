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

const int RADIO_TX_PIN = -1; // TODO: put the actual pin number
const int RADIO_RX_PIN = -1;

const int GPS_TX_PIN = -1;
const int GPS_RX_PIN = -1;

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
SerialBus radio_bus(RADIO_RX_PIN, RADIO_TX_PIN, RADIO_BAUD_RATE);
SerialBus gps_bus(GPS_RX_PIN, GPS_TX_PIN, GPS_BAUD_RATE);

TeensyTime imu_time;
Adafruit_LIS2MDL mag;

//I2CBus magnetometer;
//I2CBus barometer;

LSM6DSV80X::IMU_Data imu_data;
RFD900XUS::telemetry_packet telemetry;

LSM6DSV80X imu(imu_bus, imu_time);
RFD900XUS radio(radio_bus);

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
  radio_bus.begin(); // Radio
  gps_bus.begin(); // GPS
  imu.begin(); // IMU
//  magnetometer.begin() // Magnetometer
//  barometer.begin() // Barometer

  initSD(ssPin); // Returns Error if Failed.      

}

void loop() {
  now = micros();

  if (now - prev >= DELAY) {

    // Just here to test, remove later
    imu.read(imu_data); // imu_data will naturally be updated with the readings from the last imu measurement taken
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
          radio.transmit_to_base_station(telemetry);
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