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
#include <SD.h>
#include <FS.h>
#include <Adafruit_LIS2MDL.h>
#include <Adafruit_GPS.h>


// Header Files
#include "startup.hpp"
#include "myenums.hpp"
#include <string.h>
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
const int RADIO_BAUD_RATE = 57600;

// Declaring Constants Here
const uint32_t DELAY = 100000UL; // uS

TeensyTime time_source; 
// Declaring Variables Here
uint32_t now = time_source.now_us();

// Periods of when to do each action
const uint32_t FILTER_PERIOD_US  = DELAY; // Do it every loop
const uint32_t GPS_PERIOD_US     = 1000000UL; // Placeholders
const uint32_t MAG_PERIOD_US     = 50000UL;
const uint32_t BARO_PERIOD_US    = 50000UL;
const uint32_t IMU_PERIOD_US     = DELAY;
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
uint32_t time_imu_prev = now;

const float FILTER_GYRO_WEIGHT = 0.5f;
const float FILTER_ACCEL_WEIGHT = 0.5f; // Must add to 1.0

uint32_t prev = now;
float mag_dec = 0.0f;

// Create Objects Here
SPIBus imu_bus(SPI, 10);
RFD900XUS radio(Serial5);
LSM6DSV80X imu(imu_bus, time_source);
Adafruit_GPS gps(&Serial1);
SDCard sd_card(ssPin);

ComplementaryFilter filter(FILTER_GYRO_WEIGHT, FILTER_ACCEL_WEIGHT);
Filter::Prediction prediction;
Filter::Measurements measurements;

Adafruit_LIS2MDL mag;

//I2CBus magnetometer;
//I2CBus barometer;

LSM6DSV80X::IMU_Data imu_data;
RFD900XUS::telemetry_packet telemetry;
Calibration::Offsets offset;

controlType controls = controlType::CANARDS;
flightState state = flightState::PREFLIGHT_IDLE;

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
  //imu.begin(); // IMU
  sd_card.begin();

  //  magnetometer.begin() // Magnetometer
  //  barometer.begin() // Barometer

  Calibration calibration(radio, imu);
  calibration.get_offsets(offset); // Runs the calibration routine. We probably want to move this somewhere else

  
  // Enable RMC and GGA
  gps.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // Set the default update rate of 1HZ
  gps.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
}

void loop() {
  now = time_source.now_us();
  if (DEBUG_MODE == true) {
    Serial.print("Loop start, now = ");
    Serial.println(now);
  }


  char command_buffer[RFD900XUS::RADIO_RECEIVE_LIMIT + 1] = {0};


  char command_buffer[RFD900XUS::RADIO_RECEIVE_LIMIT + 1] = {0};

  if (radio.is_command_available()) {
      if (radio.receive_command(command_buffer)) {
          if (DEBUG_MODE == true) {
            Serial.print("Received command: ");
            Serial.println(command_buffer);
          }

          if (strcmp(command_buffer, "ARM") == 0) {
              state = flightState::POWERED_ASCENT;
              radio.send_message("ARMED");
          }
          else if (strcmp(command_buffer, "PING") == 0) {
              radio.send_message("PONG");
          }
          else if (strcmp(command_buffer, "RESET") == 0) {
              radio.send_message("RESET_OK");
          }

          command_buffer[0] = '\0';
      }
  }
      
    switch (state) {  
      case flightState::PREFLIGHT_IDLE:
   
        break;
      
      case flightState::POWERED_ASCENT: {
       


  
        // Just here to test, remove later
        if (now - time_imu_prev >= IMU_PERIOD_US) {
            time_imu_prev = time_source.now_us();
            imu.read(imu_data); // imu_data will naturally be updated with the readings from the last imu measurement taken
            Calibration::apply_offsets(offset, imu_data); // Apply the offsets in software not hardware
            
            measurements.imu = imu_data; // for filter

            if (DEBUG_MODE == true){
              Serial.print("IMU: ");
              Serial.print(imu_data.ax); Serial.print(",");
              Serial.print(imu_data.ay); Serial.print(",");
              Serial.print(imu_data.az); Serial.print(" | ");
              Serial.print(imu_data.gx); Serial.print(",");
              Serial.print(imu_data.gy); Serial.print(",");
              Serial.println(imu_data.gz);
            }
        }

        if (now - time_sd_prev >= SD_CARD_PERIOD_US) {
            if (DEBUG_MODE == true) {
              Serial.println("SD save");
            }
            time_sd_prev = time_source.now_us();
            SDCard::SD_card_data sd_data;
            sd_data.timestamp_us = now;
            sd_data.imu_data = imu_data;
            sd_data.gps_latitude = gps.latitude;
            sd_data.gps_longitude = gps.longitude;
            sd_data.gps_altitude = gps.altitude;
            sd_card.save_to_buffer(sd_data);
        }

        if (sd_card.get_buffer_count() >= BUFFER_SIZE) {
            if (DEBUG_MODE == true) {
              Serial.println("SD write");
            }
            sd_card.buffered_write();
        }


        // Get I2C Data
        if (now - time_mag_prev >= MAG_PERIOD_US) {
            time_mag_prev = time_source.now_us();

            // mag.read
        }

        if (now - time_baro_prev >= BARO_PERIOD_US) {
            time_baro_prev = time_source.now_us();
            
            // baro.read()
        }

        // ===============
        //  TODO: Add in Drivers for sensors (LSM6DSV80X, LPS22HH)
        // ===============
        
        if (now - time_gps_prev >= GPS_PERIOD_US){
          time_gps_prev = time_source.now_us();


          if (gps.newNMEAreceived()) {
            gps.parse(gps.lastNMEA());
              
          }
        }

        // State Estimation


        // Control Systems

        if (now - time_pid_prev >= PID_PERIOD_US) {
          time_pid_prev = time_source.now_us();
          // PID.control(prediction.roll) <- maybe like this in the future?
        }


        // Data Transmission
        if (now - time_radio_prev >= RADIO_PERIOD_US) {
            time_radio_prev = time_source.now_us();

            telemetry.packet++;
            telemetry.time_ms = time_source.now_us() / 1000; // Convert to milliseconds

            telemetry.yaw = 0.0f;          // just 0.0 since we not calculating these yet
            telemetry.pitch = 0.0f;        
            telemetry.roll = prediction.roll;

            telemetry.altitude_m = gps.altitude;
            telemetry.latitude = gps.latitude;
            telemetry.longitude = gps.longitude;
              
            telemetry.imu_data = imu_data; // for radio


            radio.tx_base_station(telemetry);
        }
        if (now - time_filter_prev >= FILTER_PERIOD_US) {
          time_filter_prev = time_source.now_us();
          filter.update(prediction, measurements);
        // filter.predict/update
        }
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


/*================ EXAMPLE ZONE =======================

I2CBus bus(Wire, 0x6A);
TeensyTime time;
LSM6DSV80X imu(bus, time);

SPIBus bus(SPI, 10);
TeensyTime time;
LSM6DSV80X imu(bus, time);


================ EXAMPLE ZONE =========================*/