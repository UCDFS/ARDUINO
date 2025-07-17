/**
 * @file simple_can.cpp
 * @brief Simplified CAN implementation
 */

#include "simple_can.h"

// Global instance
SimpleCAN can;

SimpleCAN::SimpleCAN() {
  // Constructor, no initialization here (do in begin())
}

bool SimpleCAN::begin(uint32_t baudrate) {
  if (debugLevel >= 1) {
    Serial.print("CAN: Initializing at ");
    Serial.print(baudrate);
    Serial.println(" bps");
  }
  
  // Initialize CAN0 on Due
  if (!Can0.begin(baudrate)) {
    if (debugLevel >= 1) {
      Serial.println("CAN: Initialization FAILED!");
    }
    return false;
  }
  
  // No filtering - accept all messages
  Can0.reset_all_mailbox();
  
  // Set up one mailbox to receive everything
  int result = Can0._setFilterSpecific(0, 0, 0, false);
  
  if (debugLevel >= 2) {
    Serial.println("CAN: Initialized successfully");
    Serial.print("CAN: Filter setup result = ");
    Serial.println(result);
  }
  
  return true;
}

void SimpleCAN::update() {
  // Check for incoming messages
  while (Can0.available() > 0) {
    CAN_FRAME frame;
    if (Can0.read(frame)) {
      // Print all received messages if debug enabled
      if (debugLevel >= 3) {
        Serial.print("CAN RX: ID=0x");
        Serial.print(frame.id, HEX);
        Serial.print(" Len=");
        Serial.print(frame.length);
        Serial.print(" Data=[");
        for (int i = 0; i < frame.length; i++) {
          if (i > 0) Serial.print(",");
          Serial.print("0x");
          if (frame.data.bytes[i] < 16) Serial.print("0");
          Serial.print(frame.data.bytes[i], HEX);
        }
        Serial.println("]");
      }
      
      // Route message based on ID
      if (frame.id == BAMOCAR_TX_ID) {
        handleBamocarMessage(frame);
      } 
      else if (frame.id == BMS_ID_PACK_INFO || frame.id == BMS_ID_LIMITS_TEMP) {
        handleBMSMessage(frame);
        // Update timestamp for comms tracking
        lastBmsMessageTime = millis();
      }
      // Add other device IDs as needed
    }
  }
  
  // Print status periodically if enabled
  if (debugLevel >= 2) {
    unsigned long now = millis();
    if (now - lastStatusPrint > 5000) { // Every 5 seconds
      Serial.println("--- CAN Status ---");
      Serial.print("Motor: ");
      Serial.print(motorRPM);
      Serial.print(" RPM, ");
      Serial.print(motorTorque);
      Serial.print(" Nm, ");
      Serial.print(motorTemp);
      Serial.println("°C");
      
      Serial.print("Battery: ");
      Serial.print(packVoltage);
      Serial.print("V, ");
      Serial.print(packCurrent);
      Serial.print("A, SOC ");
      Serial.print(packSOC);
      Serial.print("%, DCL ");
      Serial.print(packDCL);
      Serial.println("A");
      
      Serial.print("BMS Temps: High ");
      Serial.print(highTemp);
      Serial.print("°C, Low ");
      Serial.print(lowTemp);
      Serial.print("°C, Relay State 0x");
      Serial.println(relayState, HEX);
      
      lastStatusPrint = now;
    }
  }
}

bool SimpleCAN::send(uint32_t id, uint8_t length, uint8_t *data) {
  CAN_FRAME frame;
  frame.id = id;
  frame.length = length;
  frame.extended = false;
  
  // Copy data bytes
  for (int i = 0; i < length; i++) {
    frame.data.bytes[i] = data[i];
  }
  
  if (debugLevel >= 3) {
    Serial.print("CAN TX: ID=0x");
    Serial.print(id, HEX);
    Serial.print(" Len=");
    Serial.print(length);
    Serial.print(" Data=[");
    for (int i = 0; i < length; i++) {
      if (i > 0) Serial.print(",");
      Serial.print("0x");
      if (data[i] < 16) Serial.print("0");
      Serial.print(data[i], HEX);
    }
    Serial.println("]");
  }
  
  bool result = Can0.sendFrame(frame);
  
  if (!result && debugLevel >= 1) {
    Serial.println("CAN: Send failed!");
  }
  
  return result;
}

bool SimpleCAN::sendBamocarCmd(uint8_t regID, uint16_t value) {
  uint8_t data[3];
  data[0] = regID;                // Register ID
  data[1] = value & 0xFF;         // Low byte
  data[2] = (value >> 8) & 0xFF;  // High byte
  
  return send(BAMOCAR_RX_ID, 3, data);
}

bool SimpleCAN::requestBamocarData(uint8_t regID, uint8_t interval) {
  uint8_t data[3];
  data[0] = 0x3D;     // REG_REQUEST
  data[1] = regID;    // Register to request
  data[2] = interval; // Update interval
  
  return send(BAMOCAR_RX_ID, 3, data);
}

void SimpleCAN::handleBamocarMessage(const CAN_FRAME &frame) {
  if (frame.length < 1) return;
  
  uint8_t regID = frame.data.bytes[0];
  
  // Basic parsing of common registers
  switch (regID) {
    case 0x30: { // N_ACTUAL (Speed)
      if (frame.length >= 3) {
        int16_t raw = frame.data.bytes[1] | (frame.data.bytes[2] << 8);
        // Convert to actual RPM based on N_MAX (might need lookup)
        motorRPM = raw * 0.5f; // Example scaling, adjust as needed
      }
      break;
    }
    
    case 0x90: { // TORQUE
      if (frame.length >= 3) {
        int16_t raw = frame.data.bytes[1] | (frame.data.bytes[2] << 8);
        motorTorque = raw * 0.01f; // Example scaling
      }
      break;
    }
    
    case 0x49: { // TEMP_MOTOR
      if (frame.length >= 3) {
        uint16_t raw = frame.data.bytes[1] | (frame.data.bytes[2] << 8);
        motorTemp = raw * 0.1f; // Example: temp in degrees C
      }
      break;
    }
    
    case 0x40: { // STATUS
      if (frame.length >= 5) {
        uint32_t status = frame.data.bytes[1] | 
                         (frame.data.bytes[2] << 8) | 
                         (frame.data.bytes[3] << 16) | 
                         (frame.data.bytes[4] << 24);
        
        // Example status bit check
        systemError = (status & 0x01) != 0;
        
        if (debugLevel >= 2 && systemError) {
          Serial.print("Bamocar ERROR status: 0x");
          Serial.println(status, HEX);
        }
      }
      break;
    }
  }
}

void SimpleCAN::handleBMSMessage(const CAN_FRAME &frame) {
  switch (frame.id) {
    case BMS_ID_PACK_INFO: { // 0x6B0
      if (frame.length == 8) {
        // Extract values without checksum validation for now
        
        // Pack Current (BYTE0) - May need scaling
        // Assuming this is a signed value representing amps
        packCurrent = (int8_t)frame.data.bytes[0];
        
        // Pack Voltage (BYTE2) - May need scaling
        // Typical scaling might be *0.1 or *0.5 for volts
        packVoltage = frame.data.bytes[2];
        
        // Pack SOC (BYTE4) - Likely percentage 0-100
        packSOC = frame.data.bytes[4];
        
        // Relay State (BYTE5) - Bitmap of relay states
        relayState = frame.data.bytes[5];
        
        if (debugLevel >= 3) {
          Serial.println("BMS Pack Info received (no checksum validation)");
        }
      }
      break;
    }
    
    case BMS_ID_LIMITS_TEMP: { // 0x6B1
      if (frame.length == 8) {
        // Extract values without checksum validation
        
        // Pack DCL (BYTE0) - Discharge Current Limit
        packDCL = frame.data.bytes[0];
        
        // Temperature values (BYTE4, BYTE5)
        highTemp = (int8_t)frame.data.bytes[4];
        lowTemp = (int8_t)frame.data.bytes[5];
        
        if (debugLevel >= 3) {
          Serial.println("BMS Limits/Temp received (no checksum validation)");
        }
      }
      break;
    }
  }
}