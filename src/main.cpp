/**
 * @file main.cpp
 * @brief Main application file for the UCD Formula Student EV Controller.
 * Initializes hardware and runs the main control loop.
 * @author Shane Whelan (UCD Formula Student)
 * @date 2025-05-23
 */

#include "header.h" // Includes all necessary headers, including new function prototypes

// Global variables defined elsewhere 
extern int brakePressure; // Defined in brake_light.cpp
// DEBUG_MODE is now globally defined in debug_cycler.cpp

// MPU object and initialization flag (defined here)
Adafruit_MPU6050 mpu;
bool mpuInitialized = false;

//------------------------------------------------------------------------------
// SETUP FUNCTION
//------------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 5000); 

  setup_debug_cycler(); // Initialize debug mode cycling system (this will print initial mode)

  // Initial boot message, respects current DEBUG_MODE value set by debug_cycler
  if (DEBUG_MODE > 0) { 
    Serial.println("--- UCD FS EV Controller Booting ---");
    if (BENCH_TEST_MODE) { // BENCH_TEST_MODE is from config_testing.h
      Serial.println("*** BENCH TEST MODE ENABLED ***");
    }
  }

  // --- Initialize Pins ---
  pinMode(BRAKE_LIGHT_PIN, OUTPUT);
  digitalWrite(BRAKE_LIGHT_PIN, LOW);
  monitor_errors_setup();

  // --- Initialize I2C for MPU6050 ---
  if (DEBUG_MODE > 0) {
    Serial.println("MAIN: Initializing I2C bus (Wire)...");
  }
  Wire.begin(); 
  delay(100);   

  initializeMPU(); 

  // --- Initialize CAN Communication ---
  if (!BENCH_TEST_MODE) {
    if (!can_manager.initialize(CAN_BPS_500K)) {
      if (Serial) Serial.println("FATAL: CAN Initialization failed! Halting.");
      while (1);
    }
  } else {
    if (DEBUG_MODE > 0 && Serial) {
      Serial.println("MAIN: BENCH_TEST_MODE - CAN Initialization bypassed.");
    }
  }

  // dash_setup(); 

  if (!BENCH_TEST_MODE) {
    bamocar.requestStatus(INTVL_IMMEDIATE);
  }

  if (DEBUG_MODE > 0 && Serial) {
    Serial.println("--- Setup Complete ---");
  }
}

//------------------------------------------------------------------------------
// MAIN LOOP
//------------------------------------------------------------------------------
void loop() {
  loop_check_debug_cycler(); // Check for debug mode changes via button

  if (!BENCH_TEST_MODE) {
    can_manager.process_incoming_messages();
  }

  brake_light(); 
  monitor_errors_loop(); 
  motor_control_update();
  // dash_loop(); 

  if (DEBUG_MODE >= 3) { 
    static unsigned long last_debug_print = 0;
    if (millis() - last_debug_print > 1000) { 
      if (Serial) { 
        Serial.println("\n--- Loop Status Update ---");
        double current_apps_percent = get_apps_reading();
        if (current_apps_percent < 0.0) {
          Serial.println("  APPS: IMPLAUSIBLE");
        } else {
          Serial.print("  APPS Percent: ");
          Serial.print(current_apps_percent, 1);
          Serial.println("%");
        }
        Serial.print("  Brake Pressure (Raw): ");
        Serial.println(brakePressure);
        if (!BENCH_TEST_MODE) {
          const BMSData &bms_data = bms_handler.get_bms_data();
          Serial.print("  BMS SoC: ");
          Serial.print(bms_data.pack_soc, 1);
          Serial.println("%");
          Serial.print("  BMS Voltage: ");
          Serial.print(bms_data.pack_voltage, 2);
          Serial.println(" V");
          Serial.print("  BMS Fault: ");
          Serial.println(bms_handler.has_critical_fault() ? "YES" : "NO");
          Serial.print("  Bamocar Status: 0x");
          Serial.println(bamocar.getStatus(), HEX);
        } else {
          Serial.println("  (BMS/Bamocar real-time data bypassed in BENCH_TEST_MODE)");
        }
        Serial.println("--------------------------");
      }
      last_debug_print = millis();
    }
  }
}
