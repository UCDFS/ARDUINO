/**
 * @file simple_can.h
 * @brief Simplified CAN interface for Formula Student VCU
 * @author UCD Formula Student
 */

#pragma once

#include <due_can.h>
#include <Arduino.h>
#include "header.h"

// CAN ID definitions
// Bamocar D3 default IDs
#define BAMOCAR_RX_ID 0x201  // ID to send TO Bamocar
#define BAMOCAR_TX_ID 0x181  // ID to receive FROM Bamocar

// BMS IDs - ACTUAL values from BMS utility
#define BMS_ID_PACK_INFO   0x6B0  // Pack current, voltage, SOC, relay state
#define BMS_ID_LIMITS_TEMP 0x6B1  // DCL and temperatures

class SimpleCAN {
public:
  SimpleCAN();
  
  /**
   * @brief Initialize CAN bus
   * @param baudrate The CAN baud rate (e.g., 500000)
   * @return true if initialized successfully
   */
  bool begin(uint32_t baudrate);
  
  /**
   * @brief Process any incoming CAN messages
   * Called regularly from main loop
   */
  void update();
  
  /**
   * @brief Send a CAN message
   * @param id The CAN ID
   * @param length Data length (1-8)
   * @param data Pointer to data bytes
   * @return true if sent successfully
   */
  bool send(uint32_t id, uint8_t length, uint8_t *data);
  
  /**
   * @brief Send a register command to Bamocar
   * @param regID Register ID
   * @param value 16-bit value to send
   * @return true if sent successfully
   */
  bool sendBamocarCmd(uint8_t regID, uint16_t value);
  
  /**
   * @brief Request data from Bamocar
   * @param regID Register to request
   * @param interval Update interval (0=once)
   * @return true if request sent successfully
   */
  bool requestBamocarData(uint8_t regID, uint8_t interval);
  
  // Add debug level control
  void setDebugLevel(uint8_t level) { debugLevel = level; }
  
  // Getters for vehicle data
  float getMotorRPM() const { return motorRPM; }
  float getMotorTorque() const { return motorTorque; }
  float getMotorTemp() const { return motorTemp; }
  float getPackVoltage() const { return packVoltage; }
  float getPackCurrent() const { return packCurrent; }
  float getPackSOC() const { return packSOC; }
  float getPackDCL() const { return packDCL; }
  uint8_t getRelayState() const { return relayState; }
  int8_t getHighTemp() const { return highTemp; }
  int8_t getLowTemp() const { return lowTemp; }
  bool getSystemError() const { return systemError; }
  bool getBmsCommsActive() const { 
    return (millis() - lastBmsMessageTime) < 3000; 
  }
  
private:
  uint8_t debugLevel = 0; // 0=None, 1=Errors, 2=Basic, 3=Verbose
  unsigned long lastStatusPrint = 0;
  unsigned long lastBmsMessageTime = 0;
  
  // Callback functions for different message types
  void handleBamocarMessage(const CAN_FRAME &frame);
  void handleBMSMessage(const CAN_FRAME &frame);
  
  // Vehicle state variables
  // Motor controller data
  float motorRPM = 0;
  float motorTorque = 0;
  float motorTemp = 0;
  bool systemError = false;
  
  // BMS data
  float packVoltage = 0;
  float packCurrent = 0;
  float packSOC = 0;
  float packDCL = 0;
  uint8_t relayState = 0;
  int8_t highTemp = 0;
  int8_t lowTemp = 0;
};

// Global instance
extern SimpleCAN can;