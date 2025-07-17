/**
 * @file brake_light.cpp
 * @brief Handles brake light activation based on brake pressure and deceleration
 * @author Shane Whelan (UCD Formula Student)
 * @date 2025-04-27
 */

#include "header.h"

// Global brake pressure variables (raw ADC values)
int brakePressureFront = 0;
int brakePressureRear = 0; 
int brakePressureCombined = 0;

// Brake initialization variables
int brakeIdleValueFront = 0;
int brakeIdleValueRear = 0;
int brakeIdleValueCombined = 0;
int dynamicBrakeThreshold = 0;
bool brakeInitialized = false;
const int BRAKE_THRESHOLD_DELTA = 10; // Threshold above idle value

extern Adafruit_MPU6050 mpu;
extern bool mpuInitialized;

bool initializeMPU() {
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 sensor!");
    return false;
  } else {
    if (DEBUG_MODE) {
      Serial.println("MPU6050 sensor initialized.");
    }
    mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  }
  return true;
}

void brake_light() {
  // Read brake pressure from both sensors
  brakePressureRear = analogRead(BRAKE_PRESSURE_SENSOR_PIN_REAR);
  brakePressureFront = analogRead(BRAKE_PRESSURE_SENSOR_PIN_FRONT);
  brakePressureCombined = (brakePressureFront + brakePressureRear) / 2;

  // Initialize brake idle values if needed
  if (!brakeInitialized) {
    brakeIdleValueFront = brakePressureFront;
    brakeIdleValueRear = brakePressureRear;
    brakeIdleValueCombined = brakePressureCombined;
    dynamicBrakeThreshold = brakeIdleValueCombined + BRAKE_THRESHOLD_DELTA;
    brakeInitialized = true;
    
    if (DEBUG_MODE) {
      Serial.print("Brake initialized - Front idle: ");
      Serial.print(brakeIdleValueFront);
      Serial.print(", Rear idle: ");
      Serial.print(brakeIdleValueRear);
      Serial.print(", Combined idle: ");
      Serial.print(brakeIdleValueCombined);
      Serial.print(", Dynamic threshold: ");
      Serial.println(dynamicBrakeThreshold);
    }
  }

  float deceleration_m_s2 = 0.0f;

  if (mpuInitialized) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    // Calculate deceleration (assuming negative x is deceleration)
    deceleration_m_s2 = -a.acceleration.x;
  }

  // Debug output for brake pressure readings
  if (DEBUG_MODE >= 6) {
    static unsigned long lastBrakePrint = 0;
    if (millis() - lastBrakePrint > 1000) {
      Serial.print("Brake - Front: ");
      Serial.print(brakePressureFront);
      Serial.print(" (idle: ");
      Serial.print(brakeIdleValueFront);
      Serial.print("), Rear: ");
      Serial.print(brakePressureRear);
      Serial.print(" (idle: ");
      Serial.print(brakeIdleValueRear);
      Serial.print("), Combined: ");
      Serial.print(brakePressureCombined);
      Serial.print(" (threshold: ");
      Serial.print(dynamicBrakeThreshold);
      Serial.println(")");
      lastBrakePrint = millis();
    }
  }

  // Determine brake light state
  bool activate_brake_light = false;

  // Condition 1: Hydraulic pressure threshold using dynamic threshold
  if (brakePressureCombined > dynamicBrakeThreshold) {
    activate_brake_light = true;
  }

  // Condition 2: Deceleration threshold due to regen (Rule T6.3.1)
  const float REGEN_DECEL_THRESHOLD = 1.0f; // m/s^2
  if (deceleration_m_s2 > REGEN_DECEL_THRESHOLD) {
    activate_brake_light = true;
  //   if (DEBUG_MODE >= 2)
  //     Serial.println("Brake Light ON (Regen Decel)");
  // 
  }

  // Apply hysteresis for turning the light OFF
  static bool brake_light_on = false;
  if (activate_brake_light) {
    digitalWrite(BRAKE_LIGHT_PIN, HIGH);
    brake_light_on = true;
  } else {
    bool turn_off = brakePressureCombined < (dynamicBrakeThreshold - BRAKE_LIGHT_HYSTERESIS);
    
    if (brake_light_on && turn_off) {
      digitalWrite(BRAKE_LIGHT_PIN, LOW);
      brake_light_on = false;
      // if (DEBUG_MODE)
      //   Serial.println("Brake Light OFF");
    }
  }
}

void recalibrate_brake_idle() {
  // Force recalibration by reading current values as new idle
  brakePressureRear = analogRead(BRAKE_PRESSURE_SENSOR_PIN_REAR);
  brakePressureFront = analogRead(BRAKE_PRESSURE_SENSOR_PIN_FRONT);
  brakePressureCombined = (brakePressureFront + brakePressureRear) / 2;
  
  brakeIdleValueFront = brakePressureFront;
  brakeIdleValueRear = brakePressureRear;
  brakeIdleValueCombined = brakePressureCombined;
  dynamicBrakeThreshold = brakeIdleValueCombined + BRAKE_THRESHOLD_DELTA;
  
  if (DEBUG_MODE) {
    Serial.print("Brake recalibrated - Front idle: ");
    Serial.print(brakeIdleValueFront);
    Serial.print(", Rear idle: ");
    Serial.print(brakeIdleValueRear);
    Serial.print(", Combined idle: ");
    Serial.print(brakeIdleValueCombined);
    Serial.print(", New threshold: ");
    Serial.println(dynamicBrakeThreshold);
  }
}