/**
 * @file header.h
 * @brief Central header file including libraries, constants, pin definitions,
 * and function prototypes.
 * @author Shane Whelan (UCD Formula Student)
 * @date 2025-04-27
 */

// TODO:
// - Calibrate BRAKE_LIGHT_THRESHOLD based on sensor readings and desired light activation point.
// - Calibrate BRAKE_LIGHT_HYSTERESIS for desired behavior.
// - Verify the necessity and logic of using TILT_THRESHOLD_DEG for brake light activation;
//   consider using deceleration directly from MPU if required by rules (T6.3.1).
// - Define pins and logic for monitoring critical errors (IMD, BSPD etc.) via digital inputs.

#ifndef HEADER_H
#define HEADER_H

// ------------ ARDUINO LIBRARIES & MACRO UNDEFS ------------
#include <Arduino.h> // Include Arduino core first
#include <SPI.h>     // Required by due_can? Keep if needed.
#include <Wire.h>    // For I2C devices like MPU6050
#include <due_can.h> // This includes Arduino.h

// Undefine min and max macros to prevent conflicts with C++ standard library
#ifdef min
#undef min
#endif
#ifdef max
#undef max
#endif

// ------------ STANDARD C++ LIBRARIES ------------
#include <limits>    // For std::numeric_limits
#include <cmath>     // For std::fabs, std::max, etc.
#include <stdint.h>  // For fixed-width integer types


// ------------ EXTERNAL LIBRARIES (Already included via Arduino.h or specific needs) ------------
#include <Nextion.h>        // For Nextion display (if used)
#include <Adafruit_MPU6050.h> // For MPU6050 sensor
#include <Adafruit_Sensor.h>  // Required by Adafruit MPU6050 library

// ------------ PROJECT MODULES ------------
#include "globals.h"       // Global variable declarations
#include "apps.h"          // APPS reading constants/functions
#include "can_manager.h"   // CAN bus manager
#include "bms_handler.h"   // BMS data handler
#include "bamocar-due.h"   // Bamocar motor controller library


// ------------ CONSTANTS ------------
// --- General ---
// DEBUG_MODE: Controls verbosity of Serial print messages.
//   0 = Off: No debug messages.
//   1 = Basic: Critical events, initialization, fault state changes.
//   2 = Detailed: More frequent status updates (e.g., APPS raw/percentage, MPU data, regen skip reasons).
//   3 = Verbose: High-frequency loop status updates from main.cpp, detailed periodic motor control status.
const int DEBUG_MODE = 3; 

// --- Pins ---
// Analog Pins
const int BRAKE_PRESSURE_SENSOR_PIN = A0;
const int APPS_1_PIN = A6;
const int APPS_2_PIN = A7;
// Digital Pins
const int BRAKE_LIGHT_PIN = 7;
// TODO: Define pins used for monitoring critical errors (IMD, BSPD, etc.)
const int ERROR_PIN_START = 22; // Example start pin for error monitoring
const int ERROR_PIN_END = 37;   // Example end pin for error monitoring
// const int IMD_FAULT_PIN = 22; // Example specific pin
// const int BSPD_FAULT_PIN = 23; // Example specific pin

// --- Thresholds & Parameters ---
// Brake System
// TODO: Calibrate these thresholds based on sensor readings
const int BRAKE_LIGHT_THRESHOLD = 50; // Raw ADC value - Calibrate!
const int BRAKE_LIGHT_HYSTERESIS = 20;  // Raw ADC value - Calibrate!
const float REGEN_DECEL_THRESHOLD = 100.0f; // m/s^2 for brake light activation (Rule T6.3.1)
// TODO: Verify necessity/logic/value for tilt activation
const float TILT_THRESHOLD_DEG = 5.8;   // For MPU6050 brake light activation - Verify necessity/logic (Rule T6.3.1)
const int APPS_BRAKE_PLAUSIBILITY_THRESHOLD = 25; // % APPS request threshold for brake plausibility check (Rule EV.5.7)

// APPS
const float APPS_PLAUSIBILITY_THRESHOLD = 10.0f; // % difference threshold (Rule EV.5.6)

// Timing
const unsigned long APPS_PLAUSIBILITY_TIMEOUT_MS = 100; // Max time for APPS implausibility (Rule EV.5.6.3)
const unsigned long APPS_BRAKE_PLAUSIBILITY_TIMEOUT_MS = 500; // Max time for APPS/Brake implausibility (Rule EV.2.3.1)


// ------------ GLOBAL OBJECT INSTANCES (declared extern here) ------------
// These are defined in their respective .cpp files or main.cpp
extern CANManager can_manager;
extern BMSHandler bms_handler;
extern Bamocar bamocar;
extern Adafruit_MPU6050 mpu;
extern bool mpuInitialized; // Added for MPU status
extern int brakePressure; // Defined in brake_light.cpp, used by motor_controller and dashboard


// ------------ FUNCTION PROTOTYPES ------------
void initializeMPU(); // Added prototype for MPU initialization

// --- Sensor/Input Modules ---
double get_apps_reading(); // Returns pedal position (%) or -1.0 on implausibility
void brake_light();        // Reads brake pressure, MPU, controls brake light

// --- Actuator/Control Modules ---
void motor_control_update();

// --- Monitoring/Dashboard Modules ---
void monitor_errors_setup();
void monitor_errors_loop();
void dash_setup();
void dash_loop();

#endif // HEADER_H
