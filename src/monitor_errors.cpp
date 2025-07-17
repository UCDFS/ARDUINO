/**
 * @file monitor_errors.cpp
 * @brief Monitors digital input pins (22-37) and sets corresponding global
 * error flags.
 * @author Shane Whelan (UCD Formula Student)
 * @date 2025-04-27
 */

// TODO: find pins for each error signal, possibly don't even need this file
// - Clearly define which physical error signal (e.g., IMD Fault, BSPD Fault)
//   is connected to which specific digital input pin (22-37).
// - Update motor_controller.cpp to check the relevant errorXX flags and trigger
//   zero torque for critical faults.

#include "header.h" // Includes Arduino.h for pinMode, digitalRead

// Declare individual global error flags (extern if declared in a header file)
// These flags indicate the state (HIGH = 1, LOW = 0) of the corresponding pin.
// TODO: Map these error numbers to specific fault conditions (IMD, BSPD, etc.)
int error22 = 0;
int error23 = 0;
int error24 = 0;
int error25 = 0;
int error26 = 0;
int error27 = 0;
int error28 = 0;
int error29 = 0;
int error30 = 0;
int error31 = 0;
int error32 = 0;
int error33 = 0;
int error34 = 0;
int error35 = 0;
int error36 = 0;
int error37 = 0;

//------------------------------------------------------------------------------
// Setup function for error monitoring pins
//------------------------------------------------------------------------------
/**
 * @brief Configures digital pins 22 through 37 as inputs.
 * Called once from the main setup() function.
 */
void monitor_errors_setup() {
  // Loop through the defined range of error monitoring pins
  for (int pin = ERROR_PIN_START; pin <= ERROR_PIN_END; pin++) {
    pinMode(pin, INPUT); // Set each pin as a digital input
  }
  if (DEBUG_MODE) {
    Serial.println("Error monitoring pins (22-37) initialized as inputs.");
  }
}

//------------------------------------------------------------------------------
// Loop function to read error monitoring pins
//------------------------------------------------------------------------------
/**
 * @brief Reads the state of digital pins 22 through 37 and updates
 * the corresponding global errorXX flag.
 * Called repeatedly from the main loop() function.
 */
void monitor_errors_loop() {
  // Loop through the defined range of error monitoring pins
  for (int pin = ERROR_PIN_START; pin <= ERROR_PIN_END; pin++) {
    int state = digitalRead(pin); // Read the current state of the pin

    // Update the corresponding global error flag based on the pin number
    // A HIGH state on the pin sets the flag to 1, LOW sets it to 0.
    switch (pin) {
    case 22:
      error22 = (state == HIGH) ? 1 : 0;
      break;
    case 23:
      error23 = (state == HIGH) ? 1 : 0;
      break;
    case 24:
      error24 = (state == HIGH) ? 1 : 0;
      break;
    case 25:
      error25 = (state == HIGH) ? 1 : 0;
      break;
    case 26:
      error26 = (state == HIGH) ? 1 : 0;
      break;
    case 27:
      error27 = (state == HIGH) ? 1 : 0;
      break;
    case 28:
      error28 = (state == HIGH) ? 1 : 0;
      break;
    case 29:
      error29 = (state == HIGH) ? 1 : 0;
      break;
    case 30:
      error30 = (state == HIGH) ? 1 : 0;
      break;
    case 31:
      error31 = (state == HIGH) ? 1 : 0;
      break;
    case 32:
      error32 = (state == HIGH) ? 1 : 0;
      break;
    case 33:
      error33 = (state == HIGH) ? 1 : 0;
      break;
    case 34:
      error34 = (state == HIGH) ? 1 : 0;
      break;
    case 35:
      error35 = (state == HIGH) ? 1 : 0;
      break;
    case 36:
      error36 = (state == HIGH) ? 1 : 0;
      break;
    case 37:
      error37 = (state == HIGH) ? 1 : 0;
      break;
    }
  }
}
