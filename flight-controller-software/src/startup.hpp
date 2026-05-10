#ifndef STARTUP_H
#define STARTUP_H

#include "myEnums.hpp"

/**
 * @brief Runs on startup, when prompted. Will not run on power on.
 * @returns error
 */
int startup(){
    int error = 0;

    // Test Radio

    // Test Servo / BLDC

    // Create Data Logging File on SD Card


    return error;
}


float get_mag_dec(launchSite launch_site){
    float _mag_dec = 0.0f;
    switch (launch_site)
    {
    case launchSite::IOWA_CITY:
      _mag_dec = -0.933;
      break;
    case launchSite::CEDAR_RAPIDS:
      _mag_dec = -0.633;
      break;
    case launchSite::MINNEAPOLIS:
      _mag_dec = -0.2;
      break;
    }
    return _mag_dec;
}

#endif