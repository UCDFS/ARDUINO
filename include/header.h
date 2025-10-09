/**
 * @file header.h
 * @brief Central header file including libraries, constants, pin definitions, and function prototypes(2025 update: central header file that also contains error checking and logging stuff)
 * @author Shane Whelan (UCD Formula Student)
 * @date 2025-04-27
*/

// TODO: calibration below
//   consider using deceleration directly from MPU if required by rules
//   (T6.3.1).
// - Define pins and logic for monitoring critical errors (IMD, BSPD etc.) via
// digital inputs.

#ifndef HEADER_H
#define HEADER_H

// ------------ STANDARD LIBRARIES ------------
#include <cmath>
#include <stdint.h> // For fixed-width integer types

// ------------ ARDUINO LIBRARIES ------------
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>

// Undefine Arduino's min/max macros to avoid conflicts with C++ standard library
#undef max
#undef min

#include <limits>

// ------------ EXTERNAL LIBRARIES ------------
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Nextion.h>
#include <due_can.h>

// ------------ PROJECT MODULES ------------
#include "apps.h"        // APPS reading constants/functions
#include "bamocar-due.h" // Bamocar motor controller library
#include "simple_can.h"  // For standardising a central module for CAN comms
#include "bpps.h"      // Brake pedal position sensor
#include "dashboard.h"   // Dashboard display functions
#include "error_monitor.h" // Error monitoring functions

// ------------ GLOBAL VARIABLES ------------ Add to bamocar-due.h?
extern int vehicleSpeed;
extern int motorRPM;
extern float batteryVoltage;
extern int motorTemperature;


// ------------ FUNCTION PROTOTYPES ------------

// --- Actuator/Control Modules ---
void motor_control_update(); // Handle motor control logic including safety checks // add to bamocar-due.h?

#endif // HEADER_H
 