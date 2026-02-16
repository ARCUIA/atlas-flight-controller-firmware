#ifndef MYENUMS_H
#define MYENUMS_H

// Perform different loops based on flight state
enum class flightState
{
    PREFLIGHT_IDLE,
    POWERED_ASCENT,
    UNPOWERED_ASCENT,
    CHUTE_DESCENT,
    POSTFLIGHT_IDLE
};


// To determine controls and tests
enum class controlType
{
    CANARDS,
    REACTION_WHEEL
};


// To determine magnetic declination
enum class launchSite
{
    IOWA_CITY,
    CEDAR_RAPIDS,
    MINNEAPOLIS
};

#endif