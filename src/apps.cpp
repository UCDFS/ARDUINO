/**
 * @file apps.cpp
 * @brief Handles reading and validating Accelerator Pedal Position Sensors
 * (APPS). Modified implausibility check logic.
 * @author Shane Whelan (UCD Formula Student) // Original: Hari Mohan
 * @date 2025-04-27
 */

// TODO:
// - Accurately calibrate PEDAL_VOLTAGE_MIN and PEDAL_VOLTAGE_MAX by measuring
//   the actual voltage output from each APPS sensor at 0% and 100% pedal
//   travel.
// - Verify ADC_MAX_VALUE and ADC_REF_VOLTAGE match the Arduino Due's
// configuration
//   (Due default is 10-bit ADC (0-1023) with 3.3V reference, but can be
//   changed).

#include "header.h"
#include <cmath>  // For std::fabs
#include <limits> // For std::numeric_limits

// Define pedal calibration constants here (as they are specific to this file)
// TODO: These values MUST be calibrated on the actual vehicle.
const double PEDAL_VOLTAGE_MIN = 0; // Voltage at 0% pedal travel - CALIBRATE!
const double PEDAL_VOLTAGE_MAX =
    3.2; // Voltage at 100% pedal travel - CALIBRATE!
// TODO: Verify ADC settings for Arduino Due (default 10-bit, 3.3V ref)
const double ADC_MAX_VALUE =
    1023.0; // Max value for Arduino Due's 10-bit ADC (use 4095 for 12-bit)
const double ADC_REF_VOLTAGE = 3.3; // ADC reference voltage

// Export constants needed elsewhere (e.g., potentially for dashboard display)
// These are defined above, extern declaration allows other files to see them.
// No need to export these if only used internally here.
// extern const double PEDAL_VOLTAGE_MIN;
// extern const double PEDAL_VOLTAGE_MAX;

/**
 * @brief Reads the two APPS sensors, checks for plausibility, and returns the
 * average pedal position as a percentage (0-100).
 * @return Pedal position (0.0 to 100.0) if sensors are plausible,
 * -1.0 if an implausibility is detected according to FSUK EV.5.6.
 */
double get_apps_reading() {
  int apps_1_raw = analogRead(APPS_1_PIN);
  int apps_2_raw = analogRead(APPS_2_PIN);

  // 1. Convert raw ADC values to voltages
  double apps_1_voltage = apps_1_raw * ADC_REF_VOLTAGE / ADC_MAX_VALUE;
  double apps_2_voltage = apps_2_raw * ADC_REF_VOLTAGE / ADC_MAX_VALUE;

  // 2. Convert voltages to percentage (0-100) based on calibration
  // Ensure denominator is not zero
  double pedal_range = PEDAL_VOLTAGE_MAX - PEDAL_VOLTAGE_MIN;
  if (std::fabs(pedal_range) < std::numeric_limits<double>::epsilon()) {
    if (DEBUG_MODE) {
      Serial.println("APPS Error: PEDAL_VOLTAGE_MAX == PEDAL_VOLTAGE_MIN! "
                     "Check Calibration.");
    }
    return -1.0; // Prevent division by zero, indicate error
  }

  double apps_1_percent =
      (apps_1_voltage - PEDAL_VOLTAGE_MIN) * 100.0 / pedal_range;
  double apps_2_percent =
      (apps_2_voltage - PEDAL_VOLTAGE_MIN) * 100.0 / pedal_range;

  // Clamp values to 0-100 range after conversion
  apps_1_percent = constrain(apps_1_percent, 0.0, 100.0);
  apps_2_percent = constrain(apps_2_percent, 0.0, 100.0);

  // 3. Check for Implausibility (FSUK EV.5.6: deviation > 10%)
  // Compare the calculated percentages.
  if (std::fabs(apps_1_percent - apps_2_percent) >
      APPS_PLAUSIBILITY_THRESHOLD) {
    if (DEBUG_MODE) { // This event-driven print is fine
      Serial.print("APPS Implausibility Detected! APPS1: ");
      Serial.print(apps_1_percent, 1); // Print with 1 decimal place
      Serial.print("%, APPS2: ");
      Serial.print(apps_2_percent, 1); // Print with 1 decimal place
      Serial.println("%");
    }
    // FSUK EV.5.6.3: Torque must be cut to zero within 100ms.
    // The calling function (motor_control_update) must handle this based on the
    // -1.0 return.
    return -1.0; // Indicate implausibility
  }

  // 4. Return the average percentage if plausible
  double average_percent = (apps_1_percent + apps_2_percent) / 2.0;

  // Removed the timed DEBUG_MODE >= 3 print block from here.
  // main.cpp will handle periodic APPS status printing.

  return average_percent;
}
