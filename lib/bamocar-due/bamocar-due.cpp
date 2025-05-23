/**
 * @file bamocar-due.cpp
 * @brief Implements the Bamocar class for interacting with the Bamocar D3 controller.
 * Modified to remove CAN callback registration and use CANManager for sending.
 * @author Shane Whelan (UCD Formula Student)
 * @date 2025-04-27
 */

// TODO:
// - Verify the calculation logic and scaling factors used in getCurrent() against
//   the specific Bamocar D3 CAN documentation for registers I_ACTUAL, I_DEVICE, I_200PC.
// - Verify the exact data bytes required for the setSoftEnable() command
//   (REG_ENABLE, 0x51) based on the Bamocar D3 manual. The current implementation
//   uses example values that might not be correct.
// - Verify the scaling and interpretation of N_ACTUAL in getSpeed().
// - Verify the interpretation of response lengths in _parseMessage() if needed.
// - Implement or replace getMaxTorqueNm() with an accurate value for your motor.

#include <Arduino.h> // Explicitly include Arduino.h first
#include "bamocar-due.h" // This header is in the same directory
#include "../../include/can_manager.h" // Corrected relative path
#include "../../include/header.h"      // Corrected relative path for DEBUG_MODE etc.

// Define static instance if needed elsewhere, otherwise remove
// Bamocar* Bamocar::instance = nullptr;

//------------------------------------------------------------------------------
// Setters for CAN IDs (Unchanged)
//------------------------------------------------------------------------------
void Bamocar::setRxID(uint16_t rxID) {
    _rxID = rxID;
}

void Bamocar::setTxID(uint16_t txID) {
    _txID = txID;
}

//------------------------------------------------------------------------------
// Send CAN Message via CANManager
//------------------------------------------------------------------------------
bool Bamocar::_sendCAN(M_data m_data) {
    CAN_FRAME msg;

    msg.id     = _rxID; // Send TO the Bamocar's receive ID
    msg.length = m_data.length();
    msg.data   = m_data.getData(); // Get the BytesUnion data payload
    msg.extended = false; // Assuming standard CAN IDs

    // Use the global CANManager instance to send the message
    return can_manager.send_message(msg);
}

//------------------------------------------------------------------------------
// Request Data (Uses _sendCAN, so now uses CANManager)
//------------------------------------------------------------------------------
bool Bamocar::_requestData(uint8_t requestedRegID, uint8_t interval) {
    // REG_REQUEST (0x3D) is the command to request data transmission
    // requestedRegID is the register we want data from
    // interval is the transmission frequency code (e.g., INTVL_IMMEDIATE)
    return _sendCAN(M_data(REG_REQUEST, requestedRegID, interval));
}

//------------------------------------------------------------------------------
// Handle Incoming Frame (Public wrapper)
//------------------------------------------------------------------------------
void Bamocar::handle_incoming_frame(const CAN_FRAME &msg) {
    // Basic check: ensure the message ID matches what we expect from Bamocar
    if (msg.id == _txID) {
        _parseMessage(msg);
    } else {
        // This shouldn't happen if CANManager filters correctly, but log if it does
         if (DEBUG_MODE) {
            Serial.print("Bamocar: Received frame with unexpected ID: 0x");
            Serial.print(msg.id, HEX);
            Serial.print(" Expected: 0x");
            Serial.println(_txID, HEX);
         }
    }
}


//------------------------------------------------------------------------------
// Parse Received Message (Internal logic, largely unchanged)
//------------------------------------------------------------------------------
void Bamocar::_parseMessage(const CAN_FRAME &msg) {
    // The first byte of the data payload in a Bamocar response
    // indicates which register the data belongs to.
    uint8_t response_reg_id = msg.data.bytes[0];
    int64_t receivedData; // Use 64-bit intermediate to avoid overflow issues

    // Determine data size based on typical Bamocar response lengths
    // Note: Bamocar manual should specify exact lengths for each register response.
    // Common pattern: 3 bytes total (ID + 16-bit data) or 5 bytes total (ID + 32-bit data)
    // The M_data class used for sending implies lengths 3 or 5.
    // Responses might have different lengths (e.g., 4 or 8 bytes total frame length).
    // TODO: Verify this length assumption against Bamocar manual if issues arise.
    if (msg.length <= 4) { // Assumes RegID + 16-bit data + maybe padding/unused bytes
        receivedData = _getReceived16Bit(msg);
    } else { // Assumes RegID + 32-bit data + maybe padding/unused bytes
        receivedData = _getReceived32Bit(msg);
    }

    // Update internal state based on the register ID in the response
    switch (response_reg_id) {
        case REG_STATUS: // 0x40 - Usually 32-bit status flags
             // Ensure STATUS is uint32_t in _rcvd struct
            _rcvd.STATUS = (uint32_t)_getReceived32Bit(msg); // Re-parse as 32-bit explicitly if needed
            break;

        case REG_READY: // 0xE2 - Usually 16-bit
            _rcvd.READY = (uint16_t)receivedData;
            break;

        case REG_N_ACTUAL: // 0x30 - Actual Speed (RPM), 16-bit signed
            _rcvd.N_ACTUAL = (int16_t)receivedData;
            break;

        case REG_N_MAX: // 0xC8 - Max Speed (RPM), 16-bit signed
            _rcvd.N_MAX = (int16_t)receivedData;
            break;

        case REG_I_ACTUAL: // 0x20 - Actual Current (relative), 16-bit unsigned
            _rcvd.I_ACTUAL = (uint16_t)receivedData;
            break;

        case REG_I_DEVICE: // 0xC6 - Device Current Limit (Amps), 16-bit unsigned
            _rcvd.I_DEVICE = (uint16_t)receivedData;
            break;

        case REG_I_200PC: // 0xD9 - 200% Current Ref (relative), 16-bit unsigned
            _rcvd.I_200PC = (uint16_t)receivedData;
            break;

        case REG_TORQUE: // 0x90 - Actual Torque (relative), 16-bit signed
            _rcvd.TORQUE = (int16_t)receivedData;
            break;

        case REG_RAMP_ACC: // 0x35 - Accel Ramp, 16-bit unsigned
            _rcvd.RAMP_ACC = (uint16_t)receivedData;
            break;

        case REG_RAMP_DEC: // 0xED - Decel Ramp, 16-bit unsigned
            _rcvd.RAMP_DEC = (uint16_t)receivedData;
            break;

        case REG_TEMP_MOTOR: // 0x49 - Motor Temp (°C * 10), 16-bit unsigned
            _rcvd.TEMP_MOTOR = (uint16_t)receivedData;
            break;

        case REG_TEMP_IGBT: // 0x4A - Controller Temp (°C * 10), 16-bit unsigned
            _rcvd.TEMP_IGBT = (uint16_t)receivedData;
            break;

        case REG_TEMP_AIR: // 0x4B - Air Temp (°C * 10), 16-bit unsigned
            _rcvd.TEMP_AIR = (uint16_t)receivedData;
            break;

        case REG_HARD_ENABLED: // 0xE8 - Hardware Enable Status, 16-bit unsigned
            _rcvd.HARD_ENABLED = (uint16_t)receivedData;
            break;

        // Add cases for any other registers you are reading...

        default:
            // Ignore responses for registers we didn't request or don't handle
             if (DEBUG_MODE) {
                 Serial.print("Bamocar: Received unhandled register response ID: 0x");
                 Serial.println(response_reg_id, HEX);
             }
            break;
    }
}

//------------------------------------------------------------------------------
// Get 16/32 Bit Data from Frame (Internal logic, largely unchanged)
// Assumes Little Endian format (LSB first in bytes 1, 2, ...)
//------------------------------------------------------------------------------
int16_t Bamocar::_getReceived16Bit(const CAN_FRAME &msg) {
    // Assumes data starts at byte 1 (byte 0 is register ID)
    if (msg.length < 3) return 0; // Need at least 3 bytes (ID + 2 data bytes)
    int16_t val;
    val = msg.data.bytes[1];        // LSB
    val |= (msg.data.bytes[2] << 8); // MSB
    return val;
}

int32_t Bamocar::_getReceived32Bit(const CAN_FRAME &msg) {
    // Assumes data starts at byte 1
     if (msg.length < 5) return 0; // Need at least 5 bytes (ID + 4 data bytes)
    int32_t val;
    val = msg.data.bytes[1];         // LSB
    val |= (msg.data.bytes[2] << 8);
    val |= ((uint32_t)msg.data.bytes[3] << 16); // Use uint32_t cast for intermediate shift
    val |= ((uint32_t)msg.data.bytes[4] << 24); // MSB
    return val;
}

// ----------------------------------------------------------------------------
// --- Public Interface Function Implementations ---
// (Largely unchanged, but rely on updated _sendCAN and parsed _rcvd data)
// ----------------------------------------------------------------------------

// --- Speed ---
float Bamocar::getSpeed() {
    // TODO: Verify this calculation against Bamocar manual for N_ACTUAL scaling
    if (_rcvd.N_MAX == 0) return 0.0f; // Avoid division by zero
    // N_ACTUAL is often scaled relative to N_MAX, range +/- 32767
    return (float)_rcvd.N_MAX * ((float)_rcvd.N_ACTUAL / 32767.0f);
}

bool Bamocar::setSpeed(int16_t speed) {
    // REG_N_CMD (0x31) is the command to set speed
    return _sendCAN(M_data(REG_N_CMD, speed));
}

bool Bamocar::requestSpeed(uint8_t interval) {
    bool success = true;
    // Request N_ACTUAL (0x30)
    if (!_requestData(REG_N_ACTUAL, interval)) success = false;
    // Request N_MAX (0xC8)
    if (!_requestData(REG_N_MAX, interval)) success = false;
    return success;
}

// --- Acceleration / Deceleration ---
bool Bamocar::setAccel(int16_t accel) {
    // REG_RAMP_ACC (0x35)
    return _sendCAN(M_data(REG_RAMP_ACC, accel));
}

bool Bamocar::setDecel(int16_t decel) {
    // REG_RAMP_DEC (0xED)
    return _sendCAN(M_data(REG_RAMP_DEC, decel));
}

// --- Torque ---
float Bamocar::getTorque() {
    // REG_TORQUE (0x90) response is often scaled relative to max torque, range +/- 32760
    return (float)_rcvd.TORQUE / 32760.0f; // Returns fraction -1.0 to 1.0
}

bool Bamocar::setTorque(float torque) {
    // Clamp torque fraction to +/- 1.0 (TORQUE_MAX_PERCENT is 1.0)
    if (torque > TORQUE_MAX_PERCENT) torque = TORQUE_MAX_PERCENT;
    if (torque < -TORQUE_MAX_PERCENT) torque = -TORQUE_MAX_PERCENT; // Allow negative torque if needed

    // Convert fraction to 16-bit signed integer (scaling factor 32760)
    int16_t torque16 = (int16_t)(torque * 32760.0f);

    // REG_TORQUE (0x90) is also the command register ID for setting torque
    return _sendCAN(M_data(REG_TORQUE, torque16));
}

bool Bamocar::requestTorque(uint8_t interval) {
    // Request REG_TORQUE (0x90)
    return _requestData(REG_TORQUE, interval);
}

// --- Current ---
float Bamocar::getCurrent() {
    // TODO: *** FIX THIS CALCULATION ***
    // Verify the formula against Bamocar manual for calculating actual current
    // from I_ACTUAL, I_DEVICE, and I_200PC. Check scaling and units.
    // The original formula (2/10) * ... is likely incorrect due to integer division
    // and potentially wrong scaling factors.

    if (_rcvd.I_200PC == 0) return 0.0f; // Avoid division by zero

    // Placeholder correction - VERIFY FORMULA AND SCALING!
    float relative_current = (float)_rcvd.I_ACTUAL / (float)_rcvd.I_200PC;
    // Assuming I_DEVICE is the scaling factor in Amps for 200% current? Needs check.
    // The 0.2 factor might be wrong or context dependent.
    return (0.2f * (float)_rcvd.I_DEVICE * relative_current); // Example - Needs Verification!
}

bool Bamocar::requestCurrent(uint8_t interval) {
    bool success = true;
    // Request REG_I_ACTUAL (0x20)
    if (!_requestData(REG_I_ACTUAL, interval)) success = false;
     // Request REG_I_DEVICE (0xC6)
    if (!_requestData(REG_I_DEVICE, interval)) success = false;
     // Request REG_I_200PC (0xD9)
    if (!_requestData(REG_I_200PC, interval)) success = false;
    return success;
}

// --- Temperatures ---
// Return values are typically raw (e.g., °C * 10). Conversion can be done here or by caller.
uint16_t Bamocar::getMotorTemp() {
    return _rcvd.TEMP_MOTOR; // Raw value (e.g., degC * 10)
}

bool Bamocar::requestMotorTemp(uint8_t interval) {
    return _requestData(REG_TEMP_MOTOR, interval); // 0x49
}

uint16_t Bamocar::getControllerTemp() {
    return _rcvd.TEMP_IGBT; // Raw value (e.g., degC * 10)
}

bool Bamocar::requestControllerTemp(uint8_t interval) {
    return _requestData(REG_TEMP_IGBT, interval); // 0x4A
}

uint16_t Bamocar::getAirTemp() {
    return _rcvd.TEMP_AIR; // Raw value (e.g., degC * 10)
}

bool Bamocar::requestAirTemp(uint8_t interval) {
    return _requestData(REG_TEMP_AIR, interval); // 0x4B
}

// --- Status ---
uint32_t Bamocar::getStatus() { // Changed return type to uint32_t
    return _rcvd.STATUS;
}

bool Bamocar::requestStatus(uint8_t interval) {
    return _requestData(REG_STATUS, interval); // 0x40
}

// --- Enable ---
void Bamocar::setSoftEnable(bool enable) {
    // TODO: Verify the exact data bytes required for REG_ENABLE (0x51) command
    // based on the Bamocar D3 manual. The example values below might be incorrect.
    // Example values (VERIFY THESE):
    uint8_t enable_byte1 = 0x00; // Example: Might need specific bits set
    uint8_t enable_byte2 = enable ? 0x00 : 0x04; // Example: Bit 2 = Disable?

    // Sending as 16-bit data (byte1 = LSB, byte2 = MSB)
    uint16_t enable_data = enable_byte1 | (enable_byte2 << 8);
    _sendCAN(M_data(REG_ENABLE, enable_data));
}

bool Bamocar::getHardEnable() {
    // REG_HARD_ENABLED (0xE8) response
    // Often a bitfield, check manual. Assuming non-zero means enabled.
    return (_rcvd.HARD_ENABLED != 0);
}

bool Bamocar::requestHardEnabled(uint8_t interval) {
    return _requestData(REG_HARD_ENABLED, interval); // 0xE8
}

// Helper function placeholder - replace with actual Bamocar method or constant
float Bamocar::getMaxTorqueNm() {
    // TODO: Implement this or replace with a constant representing the
    // motor's nominal maximum torque in Nm used for scaling the fraction.
    // This value is needed to convert the calculated Nm torque limit back to a fraction.
    // Example:
    return 80.0f; // Replace with your motor's actual max torque capability in Nm
}
