/**
 * @file main.cpp
 * @brief Main application file for the UCD Formula Student EV Controller.
 * Initializes hardware and runs the main control loop.
 * Modified to use CANManager and new control structure.
 * @author Shane Whelan (UCD Formula Student)
 * @date 2025-04-27
 */

#include "bamocar-due.h"
#include "bms_handler.h"
#include "can_manager.h"
#include "header.h"
#include "config_testing.h" // Include the new testing configuration header

// Global variables defined elsewhere (e.g., globals.h, brake_light.cpp)
extern int brakePressure; // Assuming brake_light.cpp defines and updates this

// Define MPU object if used globally (e.g., for brake light tilt)
Adafruit_MPU6050 mpu; // Define it here
bool mpuInitialized = false; // Definition for the MPU initialization flag

//------------------------------------------------------------------------------
// SETUP FUNCTION
//------------------------------------------------------------------------------
void setup() {
  // --- Initialize Serial Communication ---
  Serial.begin(115200); // Use a faster baud rate if possible
  while (!Serial && millis() < 5000)
    ; // Wait for serial port to connect (max 5s)
  if (DEBUG_MODE) {
    Serial.println("--- UCD FS EV Controller Booting ---");
    if (BENCH_TEST_MODE) {
      Serial.println("*** BENCH TEST MODE ENABLED ***");
      Serial.println("*** Certain hardware checks and comms will be bypassed. ***");
      Serial.println("*** ENSURE THIS IS DISABLED FOR VEHICLE OPERATION. ***");
    }
  }

  // --- Initialize Pins ---
  pinMode(BRAKE_LIGHT_PIN, OUTPUT);
  digitalWrite(BRAKE_LIGHT_PIN, LOW); // Ensure brake light is off initially

  // Initialize APPS pins (analog inputs, no pinMode needed)
  // Initialize Brake Pressure Sensor pin (analog input, no pinMode needed)

  // Initialize error monitoring pins
  monitor_errors_setup(); // Sets pins 22-37 as INPUT

  // --- Initialize CAN Communication ---
  if (!BENCH_TEST_MODE) {
    // CANManager handles CAN0.begin() and filter setup
    if (!can_manager.initialize(CAN_BPS_500K)) {
      Serial.println("FATAL: CAN Initialization failed! Halting.");
      while (1)
        ; // Halt execution
    }
  } else {
    if (DEBUG_MODE) {
      Serial.println("MAIN: BENCH_TEST_MODE - CAN Initialization bypassed.");
    }
  }


  // --- Initialize Sensors ---
  // Initialize MPU6050 (if used, e.g., in brake_light.cpp)
  // It's better to initialize here than in the loop function.
  initializeMPU(); // Call the MPU init function

  // --- Initialize Dashboard (Optional) ---
  // dash_setup(); // Uncomment if using Nextion display

  // --- Initial Requests for Device Status (Optional) ---
  if (!BENCH_TEST_MODE) {
    // Request initial status from Bamocar and BMS if needed at startup
    // Note: Periodic requests are handled in motor_control_update()
    bamocar.requestStatus(INTVL_IMMEDIATE);
    // Add BMS initial requests if applicable/needed
  }


  if (DEBUG_MODE) {
    Serial.println("--- Setup Complete ---");
  }
}

//------------------------------------------------------------------------------
// MAIN LOOP
//------------------------------------------------------------------------------
void loop() {
  // --- 1. Process Incoming CAN Messages ---
  if (!BENCH_TEST_MODE) {
    // Reads messages from CAN buffer and dispatches to handlers (BMS, Bamocar)
    can_manager.process_incoming_messages();
  }

  // --- 2. Read Sensors & Update Local States ---
  // Reads brake pressure ADC, MPU6050 (if used), updates brake light state
  brake_light(); // Updates global 'brakePressure' variable

  // Monitor error input pins
  monitor_errors_loop(); // Updates global errorXX flags

  // --- 3. Execute Core Control Logic ---
  // Reads APPS, performs safety checks (APPS plausibility, APPS/Brake, BMS
  // status), determines final torque command, and sends it via CANManager. Also
  // handles periodic CAN requests (status, temp) to Bamocar.
  motor_control_update(); // This will have its own rate-limited prints if DEBUG_MODE is high enough

  // --- 4. Update Dashboard (Optional) ---
  // dash_loop(); // Uncomment if using Nextion display

  // --- 5. Debug Output ---
  if (DEBUG_MODE >= 3) { // This is the main verbose, periodic status block
    static unsigned long last_debug_print = 0;
    if (millis() - last_debug_print > 1000) { // Print status every second
      Serial.println("\n--- Loop Status Update ---"); // Add a newline for better separation

      // APPS Data
      double current_apps_percent = get_apps_reading(); // Call it here to get current value
      if (current_apps_percent < 0.0) {
        Serial.println("  APPS: IMPLAUSIBLE");
      } else {
        Serial.print("  APPS Percent: ");
        Serial.print(current_apps_percent, 1); // Print with 1 decimal place
        Serial.println("%");
      }

      // Brake Pressure
      Serial.print("  Brake Pressure (Raw): ");
      Serial.println(brakePressure);

      // BMS/Bamocar Status (conditional on BENCH_TEST_MODE)
      if (!BENCH_TEST_MODE) {
        const BMSData &bms_data = bms_handler.get_bms_data();
        Serial.print("  BMS SoC: ");
        Serial.print(bms_data.pack_soc, 1); // 1 decimal place
        Serial.println("%");
        Serial.print("  BMS Voltage: ");
        Serial.print(bms_data.pack_voltage, 2); // 2 decimal places
        Serial.println(" V");
        Serial.print("  BMS Fault: ");
        Serial.println(bms_handler.has_critical_fault() ? "YES" : "NO");
        Serial.print("  Bamocar Status: 0x");
        Serial.println(bamocar.getStatus(), HEX);
      } else {
        Serial.println("  (BMS/Bamocar real-time data bypassed in BENCH_TEST_MODE)");
        // The motor_controller.cpp will print its "Torque Command Would Be" message
        // on its own timer if BENCH_TEST_MODE is true and DEBUG_MODE is high.
      }

      // Add more key variables here as needed...
      // For example, you could have motor_control_update return final_torque_fraction
      // or store it in a global to print it here too.

      Serial.println("--------------------------");
      last_debug_print = millis();
    }
  }

} // End of loop()
