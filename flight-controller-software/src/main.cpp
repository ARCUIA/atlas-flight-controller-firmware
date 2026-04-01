/**
 * @author Sam Manley
 * @brief Flight Controller Software for ARCUIA 2026 Secret Message Rocket
 * @date 2-15-26
 * @version 1.0.0
 */

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
#include "myEnums.hpp"
#include "sd_handler.hpp"
#include "../lib/LSM6DSV80X/LSM6DSV80X.h"
#include "../lib/Platform_Teensy/TeensyTime.hpp"
#include "../lib/Platform_Teensy/I2CBus.hpp"
#include "../lib/Ahrs/Calibration.h"

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
const int FILTER_PERIOD_US = -1;
const int GPS_PERIOD_US = -1;
const int MAG_PERIOD_US = -1; // TODO: Put the actual periods
const int BARO_PERIOD_US = -1;
const int PID_PERIOD_US = -1;
const int RADIO_PERIOD_US = -1;

// Starting times to decide when to do actions
uint32_t time_filter_prev = now;
uint32_t time_gps_prev = now;
uint32_t time_mag_prev = now;
uint32_t time_baro_prev = now;
uint32_t time_pid_prev = now;

uint32_t prev = now;
float mag_dec = 0.0f;

LSM6DSV80X::IMU_Data imu_data;

// Create Objects Here
I2CBus imu_bus(Wire,0x6A);
SerialBus radio_bus(RADIO_RX_PIN, RADIO_TX_PIN, RADIO_BAUD_RATE);
SerialBus gps_bus(GPS_RX_PIN, GPS_TX_PIN, GPS_BAUD_RATE);

TeensyTime imu_time;
Adafruit_LIS2MDL mag;

//I2CBus magnetometer;
//I2CBus barometer;

LSM6DSV80X imu(imu_bus, imu_time);
flightState state;
controlType controls = controlType::CANARDS;

void setup()
{
  // Keep Tests out of setup() in case of temp black/brownout.

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
    imu.read(imu_data);

    
    switch (state) {  
      case flightState::PREFLIGHT_IDLE:
        break;
      
      case flightState::POWERED_ASCENT:
        // Get I2C Data
        if (now - time_mag_prev >= MAG_PERIOD_US) {
            time_mag_prev += MAG_PERIOD_US;
        }

        if (now - time_baro_prev >= BARO_PERIOD_US) {
            time_baro_prev += BARO_PERIOD_US;
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
          time_pid_prev += PID_PERIOD_US;
          // PID.control()
        }



        // Data Transmission
        if (now - radio_filter_prev >= RADIO_PERIOD_US) {
          time_filter_prev += FILTER_PERIOD_US;
    // filter.predict/update
        }


        // Data Logging

        if (now - time_filter_prev >= FILTER_PERIOD_US) {
          time_filter_prev += FILTER_PERIOD_US;
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