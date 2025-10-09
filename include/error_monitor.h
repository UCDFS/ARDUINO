#ifndef ERROR_MONITOR_H
#define ERROR_MONITOR_H
// ------------ CONSTANTS ------------
// --- Debug ---
const int DEBUG_MODE = 4; // 0=Off, 1=Essential, 2=Verbose, 3=Very Verbose, 4=Max Debug

// VCU-HACK: Set to true to bypass BMS checks for bench testing without a BMS.
// TODO: MUST BE FALSE FOR VEHICLE OPERATION.
const bool BENCH_TESTING_MODE = true;

// --- Pins ---
const int ERROR_PIN_START = 22; // Start pin for error monitoring
const int ERROR_PIN_END = 37;   // End pin for error monitoring

// ------------ FUNCTION PROTOTYPES ------------
void monitor_errors_setup(); // Setup error monitoring
void monitor_errors_loop();  // Error monitoring loop
void checkCriticalSystems(); // Check critical systems and update warnings
const char* getErrorString(int errorCode); // Get error string from error code



#endif // ERROR_MONITOR_H
