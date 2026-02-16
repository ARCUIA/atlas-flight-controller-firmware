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

// Pins
const int ssPin = 10;

// Declaring Constants Here
const uint32_t DELAY = 10000UL; // uS

// Declaring Variables Here
uint32_t now = micros();
uint32_t prev = now;
float mag_dec = 0.0f;

// Create Objects Here
Adafruit_LIS2MDL mag;
flightState state;
controlType controls = controlType::CANARDS;

void setup()
{
  // Keep Tests out of setup() in case of temp black/brownout.

  mag_dec = get_mag_dec(launchSite::IOWA_CITY);
  
  // Com Busses
  Wire.begin();
  Serial1.begin(57600); // GPS
  Serial7.begin(57600); // Radio, default speed up if needed for timing.
  initSD(ssPin); // Returns Error if Failed.
}

void loop() {
  now = micros();

  if (now - prev >= DELAY) {
    switch (state) {  
      case flightState::PREFLIGHT_IDLE:
        break;
      
      case flightState::POWERED_ASCENT:
        // Get I2C Data

        // ===============
        //  TODO: Add in Drivers for sensors (LSM6DSV80X, LPS22HH)
        // ===============

        // Check if GPS ready, skip otherwise
        // if (GPS.available()){
        //   // Read GPS
        // }

        // State Estimation


        // Control Systems


        // Data Transmission


        // Data Logging
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