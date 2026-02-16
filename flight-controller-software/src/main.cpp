/**
 * @author Sam Manley
 * @brief Flight Controller Software for ARCUIA 2026 Secret Message Rocket
 * @date 2-15-26
 * @version 1.0.0
 */

// Libraries
#include <Arduino.h>

// Header Files
#include "startup.h"

// Declaring Constants Here
const uint32_t DELAY = 10000UL; // uS

// Declaring Variables Here
uint32_t now = micros();
uint32_t prev = now;

// Create Objects Here

void setup()
{
  // Keep Tests out of setup() in case of temp black/brownout.

}

void loop() {
  now = micros();

  if (now - prev >= DELAY) {
    // Get I2C Data


    // Check if GPS ready, skip otherwise
    // if (GPS.available()){
    //   // Read GPS
    // }

    // State Estimation


    // Control Systems


    // Data Transmission


    // Data Logging



    prev = now;
  }
}