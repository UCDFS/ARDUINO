// dashboard_display.cpp
// Written by Shane Whelan
// UCD Formula Student

#include "header.h"

// Dashboard-specific global variables
int inverterTemperature = 0;
bool lapTimerRunning = false;
int lapMinutes = 0;
int lapSeconds = 0;
int lapMillis = 0;
int throttlePosition = 0;
float currentDraw = 0.0;
float powerOutput = 0.0;
char driveModeStr[20] = "NORMAL";
char systemStatusStr[30] = "OK";
char errorCodeStr[10] = "NONE";
bool regenActive = false;
char trackPositionStr[20] = "START";
int batterySOC = 100;
bool systemAlert = false;
bool systemWarning = false;
char alertMessageStr[50] = "";
char warningMessageStr[50] = "";
int motorTempWarningThreshold = 80;
float batteryVoltageMinThreshold = 20.0;
int systemErrorCode = 0;

// Define Nextion objects for page 2
// Text components
NexText a1 = NexText(2, 1, "a1");  
NexText a2 = NexText(2, 2, "a2");  // Regen Status
NexText a3 = NexText(2, 3, "a3");  // Battery Voltage
NexText a4 = NexText(2, 4, "a4");  // Motor Temp
NexText a5 = NexText(2, 5, "a5");  // Inverter Temp
NexText a6 = NexText(2, 6, "a6");  // Lap Time
NexText a7 = NexText(2, 7, "a7");  // Brake Pressure
NexText a8 = NexText(2, 8, "a8");  // Throttle Position
NexText a9 = NexText(2, 9, "a9");  // Current Draw
NexText a10 = NexText(2, 10, "a10"); // Power Output
NexText a11 = NexText(2, 11, "a11"); // Drive Mode
NexText a12 = NexText(2, 12, "a12"); // System Status
NexText a13 = NexText(2, 13, "a13"); // Error Code

NexText t1 = NexText(2, 14, "t1");   // System Messages
NexText t2 = NexText(2, 15, "t2");   // Run Time
NexText t4 = NexText(2, 16, "t4");   // RPM
NexText t5 = NexText(2, 17, "t5");   // Battery State of Charge

NexText d1 = NexText(2, 18, "d1");   // Speed

// Progress bars
NexProgressBar progBattery = NexProgressBar(2, 19, "j0");  // Battery Level
NexProgressBar progThrottle = NexProgressBar(2, 20, "j1");  // Throttle Position

// Nextion touch event list
NexTouch *nex_listen_list[] = { NULL };

// Variables for elapsed time
unsigned long elapsedMillis = 0;
unsigned long previousMillis = 0;

// Buffer for text conversion
char buffer[40];

void dash_setup() {
  Serial1.begin(9600);  // Serial for Nextion communication
  
  // Initialize the Nextion display
  nexInit();
  
  Serial.println("Nextion display initialized!");
}

void dash_loop() {
  // Calculate elapsed time
  unsigned long currentMillis = millis();
  elapsedMillis += currentMillis - previousMillis;
  previousMillis = currentMillis;

  // Convert elapsed time into hours, minutes, and seconds
  unsigned long totalSeconds = elapsedMillis / 1000;
  int hours = totalSeconds / 3600;
  int minutes = (totalSeconds % 3600) / 60;
  int seconds = totalSeconds % 60;
  int fracSecs = (elapsedMillis % 1000) / 10;
  
  // Update display with real vehicle data
  
  // Speed - using real vehicle speed from VCU
  sprintf(buffer, "%d km/h", vehicleSpeed);
  a1.setText(buffer);
  
  // RPM - actual motor RPM from motor controller
  sprintf(buffer, "%d", motorRPM);
  a2.setText(buffer);
  
  // Battery Voltage - from BMS
  sprintf(buffer, "%.1f V", batteryVoltage);
  a3.setText(buffer);
  
  // Motor Temperature - from motor thermistor/sensor
  sprintf(buffer, "%d C", motorTemperature);
  a4.setText(buffer);
  
  // Inverter Temperature - from inverter sensors
  sprintf(buffer, "%d C", inverterTemperature);
  a5.setText(buffer);
  
  // Lap Time - from lap timing system or GPS
  if (lapTimerRunning) {
    sprintf(buffer, "%02d:%02d.%02d", lapMinutes, lapSeconds, lapMillis/10);
  } else {
    sprintf(buffer, "--:--:--");
  }
  a6.setText(buffer);
  
  // Brake Pressure - from brake pressure sensor
  sprintf(buffer, "%d bar", brakePressureCombined);
  a7.setText(buffer);
  
  // Throttle Position - from throttle position sensor
  sprintf(buffer, "%d%%", throttlePosition);
  a8.setText(buffer);
  
  // Current Draw - from motor controller or current sensor
  sprintf(buffer, "%.1f A", currentDraw);
  a9.setText(buffer);
  
  // Power Output - calculated from voltage and current
  sprintf(buffer, "%.1f kW", powerOutput);
  a10.setText(buffer);
  
  // Drive Mode - from VCU drive mode setting
  a11.setText(driveModeStr);
  
  // System Status
  a12.setText(systemStatusStr);
  
  // Error Code
  a13.setText(errorCodeStr);
  
  // Regen Status - based on actual regen braking state
  t1.setText(regenActive ? "REGEN ON" : "REGEN OFF");
  
  // Run Time - vehicle run time since power-on
  sprintf(buffer, "%02d:%02d:%02d", hours, minutes, seconds);
  t2.setText(buffer);
  
  // Track Position - from GPS or track position system
  sprintf(buffer, "%s", trackPositionStr);
  t4.setText(buffer);
  
  // Battery State of Charge - from BMS
  sprintf(buffer, "%d%%", batterySOC);
  t5.setText(buffer);
  
  // System Messages - display critical alerts or status messages
  if (systemAlert) {
    d1.setText(alertMessageStr);
  } else if (systemWarning) {
    d1.setText(warningMessageStr);
  } else {
    d1.setText("SYSTEMS NOMINAL");
  }
  
  // Update progress bars with real data
  progBattery.setValue(batterySOC);        // Battery level progress bar
  progThrottle.setValue(throttlePosition);  // Throttle position progress bar
  
  // Check for any errors or warnings that need to be displayed
  checkCriticalSystems();
}

// Function to get error string from error code
const char* getErrorString(int errorCode) {
  switch(errorCode) {
    case 1: return "BMS_FAULT";
    case 2: return "MOTOR_TEMP";
    case 3: return "APPS_FAULT";
    case 4: return "BRAKE_FAULT";
    case 5: return "CAN_ERROR";
    default: return "UNKNOWN";
  }
}

// Function to check critical systems and update warnings
void checkCriticalSystems() {
  // Check for temperature warnings
  if (motorTemperature > motorTempWarningThreshold) {
    sprintf(alertMessageStr, "MOTOR TEMP HIGH: %dC", motorTemperature);
    systemWarning = true;
  }
  
  // Check for battery voltage warnings
  if (batteryVoltage < batteryVoltageMinThreshold) {
    sprintf(alertMessageStr, "BATTERY LOW: %.1fV", batteryVoltage);
    systemWarning = true;
  }
  
  // Check for critical errors from VCU
  if (systemErrorCode != 0) {
    sprintf(alertMessageStr, "ERROR: %s", getErrorString(systemErrorCode));
    systemAlert = true;
  }
  
  // Check for any other system issues
  // Add additional checks as needed
}