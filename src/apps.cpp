/**
 * @file apps_sensor_reader.cpp
 * @brief Handles reading and validating Accelerator Pedal Position Sensors (APPS)
 * @author Shane Whelan (UCD Formula Student)
 * @date 2025-07-17
 */

#include "header.h"
#include <cmath>  // For std::fabs
#include <limits> // For std::numeric_limits

// Define pedal calibration constants - UPDATED based on actual measurements
// These are RAW ADC values (not voltages), since values DECREASE when pressed
const int APPS1_RAW_MIN = 477;  // Value when pedal is fully pressed (100%)
const int APPS1_RAW_MAX = 702;  // Value when pedal is released (0%)
const int APPS2_RAW_MIN = 478;  // Value when pedal is fully pressed (100%)
const int APPS2_RAW_MAX = 701;  // Value when pedal is released (0%)

/**
 * @brief Reads APPS sensors, checks for plausibility, and returns average pedal position
 * @return Pedal position (0.0 to 100.0) if sensors are plausible, -1.0 if implausible
 */
double get_apps_reading() {
  int apps_1_raw = analogRead(APPS_1_PIN);
  int apps_2_raw = analogRead(APPS_2_PIN);

  // Convert raw ADC values directly to percentage (0-100) - INVERTED values
  // Note: values decrease as pedal is pressed, so we invert the calculation
  double apps_1_percent = 100.0 * (APPS1_RAW_MAX - apps_1_raw) / 
                                  (APPS1_RAW_MAX - APPS1_RAW_MIN);
  double apps_2_percent = 100.0 * (APPS2_RAW_MAX - apps_2_raw) / 
                                  (APPS2_RAW_MAX - APPS2_RAW_MIN);

  // Clamp values to 0-100 range
  apps_1_percent = constrain(apps_1_percent, 0.0, 100.0);
  apps_2_percent = constrain(apps_2_percent, 0.0, 100.0);

  // Check for implausibility (FSUK EV.5.6: deviation > 10%)
  if (std::fabs(apps_1_percent - apps_2_percent) > 
      APPS_PLAUSIBILITY_THRESHOLD) {
    if (DEBUG_MODE) {
      Serial.print("APPS Implausibility Detected! APPS1: ");
      Serial.print(apps_1_percent);
      Serial.print("%, APPS2: ");
      Serial.print(apps_2_percent);
      Serial.println("%");
    }
    return -1.0; // Indicate implausibility
  }

  // Return the average percentage if plausible
  double average_percent = (apps_1_percent + apps_2_percent) / 2.0;

  if (DEBUG_MODE >= 6) {
    static unsigned long last_apps_print = 0;
    if (millis() - last_apps_print > 1000) {
      Serial.print("APPS Readings - Raw: ");
      Serial.print(apps_1_raw);
      Serial.print(", ");
      Serial.print(apps_2_raw);
      Serial.print(" | Percent: ");
      Serial.print(apps_1_percent, 1);
      Serial.print(", ");
      Serial.print(apps_2_percent, 1);
      Serial.print(" | Avg: ");
      Serial.println(average_percent, 1);
      last_apps_print = millis();
    }
  }

  return average_percent;
}