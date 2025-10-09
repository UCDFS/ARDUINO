#pragma once

/* Accelerator Pedal Position Sensor
 See T.11.8 and T.11.9 for more info
Requirments:
- Detect implausibility where there is a 10% difference or more between the two
sensors
- Shut down main motors when implausibility exists for 100ms or more
- Fail safe (shut down main motors) when short to ground or supply, open,
mechanicaly impossible, or otherwise broken
- Fail safe within 500ms of detecting a failure

*/

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>

// ------------ EXTERNAL LIBRARIES ------------
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Nextion.h>
#include <due_can.h>

namespace APPS {

double
get_apps_reading(); // Returns pedal position (%) or -1.0 on implausibility

extern const double PEDAL_MIN;
extern const double PEDAL_MAX;
constexpr int APPS_1_PIN = A6;
constexpr int APPS_2_PIN = A7;

// Define pedal calibration constants - UPDATED based on actual measurements
// These are RAW ADC values (not voltages), since values DECREASE when pressed
constexpr int APPS1_RAW_MIN = 477; // Value when pedal is fully pressed (100%)
constexpr int APPS1_RAW_MAX = 702; // Value when pedal is released (0%)
constexpr int APPS2_RAW_MIN = 478; // Value when pedal is fully pressed (100%)
constexpr int APPS2_RAW_MAX = 701; // Value when pedal is released (0%)

// APPS
constexpr float APPS_PLAUSIBILITY_THRESHOLD = 10.0f; // T11.8.9
} // namespace APPS
