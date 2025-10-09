/**
 * @file apps_sensor_reader.cpp
 * @brief Handles reading and validating Accelerator Pedal Position Sensors
 * (APPS)
 * @author Shane Whelan (UCD Formula Student)
 * @date 2025-07-17
 */

#include "apps.h"
#include "header.h"
#include "variant.h"
#include <cmath>  // For std::fabs
#include <limits> // For std::numeric_limits
using namespace APPS;

/**
 * @brief Reads APPS sensors, checks for plausibility, and returns average pedal
 * position
 * @return Pedal position (0.0 to 100.0) if sensors are plausible, -1.0 if
 * implausible
 */
double APPS::get_apps_reading() {
  int apps_1_raw = analogRead(APPS_1_PIN);
  int apps_2_raw = analogRead(APPS_2_PIN);

  // Convert raw ADC values directly to percentage (0-100) - INVERTED values
  // Note: values decrease as pedal is pressed, so we invert the calculation
  double apps_1_percent =
      100.0 * (APPS1_RAW_MAX - apps_1_raw) / (APPS1_RAW_MAX - APPS1_RAW_MIN);
  double apps_2_percent =
      100.0 * (APPS2_RAW_MAX - apps_2_raw) / (APPS2_RAW_MAX - APPS2_RAW_MIN);

  // Clamp values to 0-100 range
  apps_1_percent = constrain(apps_1_percent, 0.0, 100.0);
  apps_2_percent = constrain(apps_2_percent, 0.0, 100.0);

  // Check for implausibility (FSUK T11.8.9: deviation > 10%)
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

  // Verify electrical plausiblity as per T11.9.2
  if (apps_1_raw > APPS1_RAW_MAX || apps_1_raw < APPS1_RAW_MIN ||
      apps_2_raw > APPS2_RAW_MAX || apps_2_raw < APPS2_RAW_MIN) {
    if (DEBUG_MODE) {
      Serial.print("APP Output Exceeds Limits!");
      Serial.print("APPS1 Raw: ");
      Serial.print(apps_1_raw);
      Serial.print(", APPS1 Max: ");
      Serial.print(APPS1_RAW_MAX);
      Serial.print(", APPS1 Min: ");
      Serial.print(APPS1_RAW_MIN);

      Serial.print(", APPS2 Raw: ");
      Serial.print(apps_2_raw);
      Serial.print(", APPS2 Max: ");
      Serial.print(APPS2_RAW_MAX);
      Serial.print(", APPS2 Min: ");
      Serial.println(APPS2_RAW_MIN);
    }
    return -1.0;
    // It should be acceptable to treat this the same as an implausibility as
    // the requirements for implausibilites are more strict than those for SCS
    // faults
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
