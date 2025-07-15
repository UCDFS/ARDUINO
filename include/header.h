/**
 * @file header.h
 * @brief Central header file including libraries, constants, pin definitions,
 * and function prototypes.
 * @author Shane Whelan (UCD Formula Student)
 * @date 2025-05-23
 */

#ifndef HEADER_H
#define HEADER_H

// ------------ ARDUINO LIBRARIES & MACRO UNDEFS ------------
#include <Arduino.h> 
#include <SPI.h>     
#include <Wire.h>    
#include <due_can.h> 

#ifdef min
#undef min
#endif
#ifdef max
#undef max
#endif

// ------------ STANDARD C++ LIBRARIES ------------
#include <limits>    
#include <cmath>     
#include <stdint.h>  

// ------------ EXTERNAL LIBRARIES ------------
#include <Nextion.h>        
#include <Adafruit_MPU6050.h> 
#include <Adafruit_Sensor.h>  

// ------------ PROJECT MODULES ------------
#include "globals.h"       
#include "apps.h"          
#include "can_manager.h"   
#include "bms_handler.h"   
#include "bamocar-due.h"   
#include "config_testing.h" // For BENCH_TEST_MODE

// ------------ CONSTANTS ------------
// --- General ---
// DEBUG_MODE: Controls verbosity of Serial print messages. (Now a global variable)
//   0 = Off: No debug messages.
//   1 = Basic: Critical events, initialization, fault state changes.
//   2 = Detailed: More frequent status updates (e.g., APPS raw/percentage, MPU data, regen skip reasons).
//   3 = Verbose: High-frequency loop status updates from main.cpp, detailed periodic motor control status.
extern int DEBUG_MODE; // Declared as extern, will be defined in debug_cycler.cpp
extern const int MAX_DEBUG_LEVEL; // Maximum level for cycling, will be defined in debug_cycler.cpp

// --- Pins ---
const int DEBUG_CYCLE_BUTTON_PIN = 6; // Pin for cycling debug mode
const int BRAKE_PRESSURE_SENSOR_PIN = A0;
const int APPS_1_PIN = A6;
const int APPS_2_PIN = A7;
const int BRAKE_LIGHT_PIN = 7;
const int ERROR_PIN_START = 22; 
const int ERROR_PIN_END = 37;   

// --- Thresholds & Parameters ---
// Brake System
const int BRAKE_LIGHT_THRESHOLD = 50; 
const int BRAKE_LIGHT_HYSTERESIS = 20;  
const float REGEN_DECEL_THRESHOLD = 1.0; // m/s^2 for brake light activation (Rule T6.3.1)
const float TILT_THRESHOLD_DEG = 5.8;   
const int APPS_BRAKE_PLAUSIBILITY_THRESHOLD = 25; 

// APPS
const float APPS_PLAUSIBILITY_THRESHOLD = 10.0f; 

// Timing
const unsigned long APPS_PLAUSIBILITY_TIMEOUT_MS = 100; 
const unsigned long APPS_BRAKE_PLAUSIBILITY_TIMEOUT_MS = 500; 

// ------------ GLOBAL OBJECT INSTANCES (declared extern here) ------------
extern CANManager can_manager;
extern BMSHandler bms_handler;
extern Bamocar bamocar;
extern Adafruit_MPU6050 mpu;
extern bool mpuInitialized; 
extern int brakePressure; 

// ------------ FUNCTION PROTOTYPES ------------
void initializeMPU(); 
double get_apps_reading(); 
void brake_light();        
void motor_control_update();
void monitor_errors_setup();
void monitor_errors_loop();
void dash_setup();
void dash_loop();

// Debug Cycler Functions (prototypes for functions in debug_cycler.cpp)
void setup_debug_cycler();
void loop_check_debug_cycler();

#endif // HEADER_H
