#ifndef BPPS_H
#define BPPS_H
/**
 * @file bpps.h
 * @brief Handles reading brake pressure sensors, MPU6050 tilt sensor, and controls brake light
 * @author Muhammad Ibrahim Mansoor (UCD Formula Student)
 * @date 2025-10-9
 */

//BPPS = Brake Pedal Position Sensor
//----------------- CONSTANTS ------------------
//PINS
// Analog Pins (INPUT)
const int BRAKE_PRESSURE_SENSOR_PIN_FRONT = A0;
const int BRAKE_PRESSURE_SENSOR_PIN_REAR = A1;

// Digital Pins (OUTPUT)
const int BRAKE_LIGHT_PIN = 7;

//GYRO
extern bool mpuInitialized;
const float TILT_THRESHOLD_DEG = 5.8; // For MPU6050 brake light activation

// Brake Light calibration constants
const int BRAKE_LIGHT_THRESHOLD = 109; // Calibrated
const int BRAKE_LIGHT_HYSTERESIS = 2; // Calibrated

//--------------- GLOBAL VARIABLES --------------------

//BRAKE PRESSURE VARIABLES
extern int brakePressureFront;
extern int brakePressureRear;
extern int brakePressureCombined;
//BRAKE IDLE VALUES
extern int brakeIdleValueFront;
extern int brakeIdleValueRear;
extern int brakeIdleValueCombined;

extern int dynamicBrakeThreshold;
extern bool brakeInitialized;

//------------ GLOBAL OBJECTS ---------------
extern Adafruit_MPU6050 mpu;
#endif // BPPS_H

//------------ FUNCTION PROTOTYPES -----------
bool initializeMPU(); // Initialize MPU6050 sensor
void brake_light(); // Reads brake pressure, MPU, controls brake light
void recalibrate_brake_idle(); // Recalibrate brake idle values
