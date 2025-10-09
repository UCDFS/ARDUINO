#pragma once
//APPS = Accelerator Pedal Position Sensor
//-----------------------CONSTANTS------------------------
extern const double PEDAL_MIN;
extern const double PEDAL_MAX;

// APPS Pins
const int APPS_1_PIN = A6;
const int APPS_2_PIN = A7;

// APPS_PLAUSIBILITY_THRESHOLDS
// Brake Sensor
const int APPS_BRAKE_PLAUSIBILITY_THRESHOLD = 25; // % APPS request threshold for brake plausibility check (Rule EV.5.7)

// APPS
const float APPS_PLAUSIBILITY_THRESHOLD = 10.0f; // % difference threshold (Rule EV.5.6)

// TIMEOUTS
const unsigned long APPS_PLAUSIBILITY_TIMEOUT_MS = 100; // Max time for APPS implausibility (Rule EV.5.6.3)
const unsigned long APPS_BRAKE_PLAUSIBILITY_TIMEOUT_MS = 500; // Max time for APPS/Brake implausibility (Rule EV.2.3.1)

//-----------------------FUNCTION PROTOTYPES------------------------
double get_apps_reading(); // Returns pedal position (%) or -1.0 on implausibility