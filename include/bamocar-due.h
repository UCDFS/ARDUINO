/**
 * @file bamocar-due.h
 * @brief Defines the Bamocar class for interacting with the Bamocar D3
 * controller. Modified to remove CAN callback registration and expose message
 * handling.
 * @author Shane Whelan (UCD Formula Student)
 * @date 2025-04-27
 */
#pragma once

#include <Arduino.h>
#ifdef min
#undef min
#endif

#ifdef max
#undef max
#endif
#include "bamocar-registers.h"
#include "due_can.h"
#include <functional>

// #define CAN_TIMEOUT 0.01
#define TORQUE_MAX_PERCENT 1.00

// Forward declaration
class CANManager;

class M_data {
public:
  // --- Constructor Overloads (Unchanged) ---
  M_data(uint8_t regID, uint16_t m_data16) {
    data.bytes[0] = regID;
    data.bytes[1] = m_data16 & 0xFF;
    data.bytes[2] = m_data16 >> 8;
    dataLength = 3;
  }

  M_data(uint8_t regID, int16_t m_data16) {
    data.bytes[0] = regID;
    data.bytes[1] = m_data16 & 0xFF;
    data.bytes[2] = m_data16 >> 8;
    dataLength = 3;
  }

  M_data(uint8_t regID, uint32_t m_data32) {
    data.bytes[0] = regID;
    data.bytes[1] = m_data32 & 0xFF;
    data.bytes[2] = (m_data32 >> 8) & 0xFF;
    data.bytes[3] = (m_data32 >> 16) & 0xFF;
    data.bytes[4] = (m_data32 >> 24) & 0xFF;
    dataLength = 5;
  }

  M_data(uint8_t regID, int32_t m_data32) {
    data.bytes[0] = regID;
    data.bytes[1] = m_data32 & 0xFF;
    data.bytes[2] = (m_data32 >> 8) & 0xFF;
    data.bytes[3] = (m_data32 >> 16) & 0xFF;
    data.bytes[4] = (m_data32 >> 24) & 0xFF;
    dataLength = 5;
  }

  M_data(uint8_t regID, uint8_t requestedRegID, uint8_t interval) {
    data.bytes[0] = regID;
    data.bytes[1] = requestedRegID;
    data.bytes[2] = interval;
    dataLength = 3;
  }
  // --- End Constructor Overloads ---

  BytesUnion getData() { return data; }

  uint8_t length() { return dataLength; }

protected:
  BytesUnion data;
  uint8_t dataLength;
};

// -------------------------------------

class Bamocar {
public:
  /**
   * @brief Constructor for Bamocar.
   * Removed mailbox parameter as callbacks are no longer used here.
   * Initializes default CAN IDs.
   */
  Bamocar() {
    // instance = this; // Keep if static instance is needed elsewhere
    _rxID = STD_RX_ID; // ID we send commands TO
    _txID = STD_TX_ID; // ID we receive responses FROM
  }

  // --- Public Interface Functions (Unchanged signatures) ---
  float getSpeed();
  bool setSpeed(int16_t speed);
  bool requestSpeed(uint8_t interval = INTVL_IMMEDIATE);

  bool setAccel(int16_t accel);
  bool setDecel(int16_t decel);

  float getTorque();
  bool setTorque(float torque);
  bool requestTorque(uint8_t interval = INTVL_IMMEDIATE);
  float getMaxTorqueNm(); // Helper function for regen scaling

  float getCurrent(); // Remember to fix implementation (2.0/10.0)
  bool requestCurrent(uint8_t interval = INTVL_IMMEDIATE);

  uint16_t getMotorTemp();
  uint16_t getControllerTemp();
  uint16_t getAirTemp();
  bool requestMotorTemp(uint8_t interval = INTVL_IMMEDIATE);
  bool requestControllerTemp(uint8_t interval = INTVL_IMMEDIATE);
  bool requestAirTemp(uint8_t interval = INTVL_IMMEDIATE);

  uint32_t getStatus(); // Changed return type to match _rcvd.STATUS
  bool requestStatus(uint8_t interval = INTVL_IMMEDIATE);

  void setSoftEnable(bool enable);
  bool getHardEnable();
  bool requestHardEnabled(uint8_t interval = INTVL_IMMEDIATE);

  void setRxID(uint16_t rxID); // ID to send commands TO
  void setTxID(uint16_t txID); // ID to receive responses FROM
  uint16_t getTxID() const {
    return _txID;
  } // Getter for CANManager filter setup
  uint16_t getRxID() const { return _rxID; } // Getter

  /**
   * @brief Public function to handle incoming CAN frames intended for this
   * Bamocar instance. Called by CANManager.
   * @param msg The received CAN_FRAME.
   */
  void handle_incoming_frame(const CAN_FRAME &msg);

protected:
  // static Bamocar *instance; // Keep if needed
  uint16_t _rxID; // ID we send commands TO
  uint16_t _txID; // ID we receive responses FROM

  // Structure to hold received data (Unchanged)
  struct _rcvd {
    int16_t N_ACTUAL = 0, N_MAX = 0, TORQUE = 0;
    uint16_t READY = 0, I_ACTUAL = 0, I_DEVICE = 0, I_200PC = 0, RAMP_ACC = 0,
             RAMP_DEC = 0, TEMP_MOTOR = 0, TEMP_IGBT = 0, TEMP_AIR = 0,
             HARD_ENABLED = 0;
    uint32_t STATUS = 0; // Ensure this is uint32_t
  } _rcvd;

  /**
   * @brief Sends a command/request to the Bamocar via the CANManager.
   * @param m_data The M_data object containing the command.
   * @return True if the message was successfully sent by CANManager, false
   * otherwise.
   */
  bool _sendCAN(M_data m_data);

  /**
   * @brief Sends a request for data transmission from the Bamocar.
   * @param requestedRegID The register ID to request.
   * @param interval The requested transmission interval code.
   * @return True if the request message was successfully sent, false otherwise.
   */
  bool _requestData(uint8_t requestedRegID, uint8_t interval = INTVL_IMMEDIATE);

  // Removed _forwardFrame as it's no longer needed for callbacks

  /**
   * @brief Parses a received CAN message and updates the internal state
   * (_rcvd). Now called by the public handle_incoming_frame.
   * @param msg The received CAN_FRAME.
   */
  void _parseMessage(const CAN_FRAME &msg); // Made const reference

  /**
   * @brief Extracts 16-bit data from a received CAN frame.
   * @param msg The received CAN_FRAME.
   * @return The extracted 16-bit value.
   */
  int16_t _getReceived16Bit(const CAN_FRAME &msg); // Made const reference

  /**
   * @brief Extracts 32-bit data from a received CAN frame.
   * @param msg The received CAN_FRAME.
   * @return The extracted 32-bit value.
   */
  int32_t _getReceived32Bit(const CAN_FRAME &msg); // Made const reference
};
