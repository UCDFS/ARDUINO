/**
 * @file brake_light.cpp
 * @brief Handles brake light activation based on brake pressure and potentially
 * tilt/deceleration from MPU6050.
 * @author Shane Whelan (UCD Formula Student)
 * @date 2025-04-27
 */

// TODO:
// - Accurately calibrate/verify the MPU sensor orientation on the vehicle to ensure
//   a.acceleration.x correctly represents longitudinal acceleration/deceleration.
// - Consider more sophisticated filtering for MPU6050 acceleration data to get a stable
//   deceleration value, especially if it's noisy.
// - Re-evaluate the TILT_THRESHOLD_DEG and its calculation if it's primarily for safety
//   or stationary conditions, as deceleration is the main dynamic factor from Rule T6.3.1.
// - Refine hysteresis values for deceleration and tilt turn-off conditions.

#include "header.h"

// Global variable for brake pressure (raw ADC) - updated here
int brakePressure = 0;

// Global MPU object (declared extern in header.h and defined in main.cpp)
extern Adafruit_MPU6050 mpu;

// MPU Initialization state flag (declared extern, defined in main.cpp)
extern bool mpuInitialized;

// Thresholds for MPU-based activation (already in header.h or defined here for clarity)
// const float REGEN_DECEL_THRESHOLD = 1.0f; // m/s^2 - Defined in header.h if shared, or define locally
// const float TILT_THRESHOLD_DEG = 5.8;    // degrees - Defined in header.h

// Hysteresis values for turning OFF MPU-based conditions
const float REGEN_DECEL_HYSTERESIS = 0.2f; // m/s^2, so turn off if decel < (REGEN_DECEL_THRESHOLD - REGEN_DECEL_HYSTERESIS)
const float TILT_HYSTERESIS_DEG = 1.0f;  // degrees, so turn off if tilt < (TILT_THRESHOLD_DEG - TILT_HYSTERESIS_DEG)


//------------------------------------------------------------------------------
// Initialize MPU6050 Sensor
//------------------------------------------------------------------------------
void initializeMPU() {
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 sensor!");
    mpuInitialized = false; // Ensure flag is false on failure
  } else {
    if (DEBUG_MODE) {
      Serial.println("MPU6050 sensor initialized.");
    }
    mpuInitialized = true; // Set this flag to true if initialization is successful
    mpu.setAccelerometerRange(MPU6050_RANGE_4_G); // Example: +/- 4G range
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ); // Example: Apply some filtering
  }
}

//------------------------------------------------------------------------------
// Brake Light Control Function
//------------------------------------------------------------------------------
void brake_light() {
  brakePressure = analogRead(BRAKE_PRESSURE_SENSOR_PIN);

  float deceleration_m_s2 = 0.0f;
  float tiltAngle = 0.0f;

  if (mpuInitialized) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    // TODO: VERIFY SENSOR ORIENTATION!
    // Assuming a.acceleration.x is longitudinal, and negative values mean deceleration
    // If positive x is deceleration, then use deceleration_m_s2 = a.acceleration.x;
    deceleration_m_s2 = -a.acceleration.x; // Example: positive if decelerating

    // Tilt calculation (angle with respect to gravity vector in the XZ or YZ plane, depending on orientation)
    // This calculates pitch if X-axis is forward, Y is sideways, Z is up/down.
    // TODO: Verify this calculation for your specific MPU mounting and desired "tilt" definition.
    float ax_g = a.acceleration.x / 9.81f;
    float ay_g = a.acceleration.y / 9.81f;
    float az_g = a.acceleration.z / 9.81f;
    tiltAngle = atan2(ax_g, sqrt(ay_g * ay_g + az_g * az_g)) * 180.0 / PI;

    if (DEBUG_MODE >= 2) {
      static unsigned long lastMPUPrint = 0;
      if (millis() - lastMPUPrint > 1000) { // Print MPU data once per second
        Serial.print("MPU Accel X: "); Serial.print(a.acceleration.x, 2);
        Serial.print(" m/s^2 -> Calc Decel: "); Serial.print(deceleration_m_s2, 2);
        Serial.print(" m/s^2 | Calc Tilt (X-YZ): "); Serial.print(tiltAngle, 1);
        Serial.println(" deg");
        lastMPUPrint = millis();
      }
    }
  }

  // Determine brake light activation conditions
  bool pressure_condition = (brakePressure > BRAKE_LIGHT_THRESHOLD);
  bool decel_condition = mpuInitialized && (deceleration_m_s2 > REGEN_DECEL_THRESHOLD);
  bool tilt_condition = mpuInitialized && (tiltAngle >= TILT_THRESHOLD_DEG); // Using the threshold from header.h

  bool activate_brake_light = pressure_condition || decel_condition || tilt_condition;

  // Control the brake light
  static bool brake_light_on = false;

  if (activate_brake_light) {
    if (!brake_light_on) { // Print only when state changes to ON
      if (DEBUG_MODE) {
        Serial.print("Brake Light ON. Conditions: Pressure="); Serial.print(pressure_condition);
        Serial.print(", Decel="); Serial.print(decel_condition);
        Serial.print(", Tilt="); Serial.println(tilt_condition);
        Serial.print("  Values: BP="); Serial.print(brakePressure);
        Serial.print(", Decel="); Serial.print(deceleration_m_s2, 2);
        Serial.print(", Tilt="); Serial.println(tiltAngle, 1);
      }
    }
    digitalWrite(BRAKE_LIGHT_PIN, HIGH);
    brake_light_on = true;
  } else { // None of the primary ON conditions are currently met
    // Check if we can turn OFF, considering hysteresis for all conditions
    bool can_turn_off_pressure = (brakePressure < (BRAKE_LIGHT_THRESHOLD - BRAKE_LIGHT_HYSTERESIS));
    bool can_turn_off_decel = !mpuInitialized || (deceleration_m_s2 < (REGEN_DECEL_THRESHOLD - REGEN_DECEL_HYSTERESIS));
    bool can_turn_off_tilt = !mpuInitialized || (tiltAngle < (TILT_THRESHOLD_DEG - TILT_HYSTERESIS_DEG));

    if (brake_light_on && can_turn_off_pressure && can_turn_off_decel && can_turn_off_tilt) {
      digitalWrite(BRAKE_LIGHT_PIN, LOW);
      brake_light_on = false;
      if (DEBUG_MODE) {
        Serial.println("Brake Light OFF");
         Serial.print("  Values: BP="); Serial.print(brakePressure);
         Serial.print(", Decel="); Serial.print(deceleration_m_s2, 2);
         Serial.print(", Tilt="); Serial.println(tiltAngle, 1);
      }
    }
    // If any of the "can_turn_off" conditions are false (e.g., still decelerating above off-threshold),
    // the light remains ON if it was already on.
  }
}
