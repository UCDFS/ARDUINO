/**
 * @file bms_handler.cpp
 * @brief Implements the BMSHandler class for processing Orion BMS 2 CAN messages.
 * @author Shane Whelan (UCD Formula Student)
 * @date 2025-04-27
 */

// TODO:
// - CRITICAL: Verify byte order (Big Endian assumption for multi-byte below)
//   and EXACT scaling factors & offsets for ALL parsed values using your
//   Orion BMS Utility and documentation. The current scaling is exemplary.
// - Implement the has_critical_fault() function based on the actual fault
//   flags and critical limits defined by the BMS and FSUK rules.
// - Parse data from Byte 6 of 0x6B0 if it contains useful information.
// - Parse data from Bytes 2 & 3 of 0x6B1 if they contain useful information.
// - Implement checksum validation for received messages if desired.

#include "bms_handler.h"
#include "can_manager.h" // For CAN_ID defines if used from there
#include "header.h"      // For DEBUG_MODE, Serial
#include <Arduino.h>     // For millis()

BMSHandler bms_handler;

BMSHandler::BMSHandler() {
    current_bms_data.pack_soc = 0.0f;
    current_bms_data.pack_voltage = 0.0f;
    current_bms_data.pack_current = 0.0f;
    current_bms_data.relay_state_raw = 0;
    current_bms_data.discharge_current_limit = 0.0f;
    current_bms_data.charge_current_limit = 0.0f;
    current_bms_data.high_cell_voltage = 0.0f;
    current_bms_data.low_cell_voltage = 5.0f;
    current_bms_data.avg_cell_voltage = 0.0f;
    current_bms_data.high_temperature = -127;
    current_bms_data.low_temperature = 127;
    current_bms_data.avg_temperature = 0;
    current_bms_data.voltage_fault = true;
    current_bms_data.temperature_fault = true;
    current_bms_data.communication_fault = true;
    current_bms_data.general_fault_code = 0xFFFF;
    current_bms_data.last_message_millis = 0;
}

void BMSHandler::handle_incoming_frame(const CAN_FRAME& frame) {
    current_bms_data.last_message_millis = millis();
    current_bms_data.communication_fault = false;

    switch (frame.id) {
        case ORION_BMS_ID_0x6B0: // Use defined ID
            parse_bms_message_0x6B0(frame);
            break;
        case ORION_BMS_ID_0x6B1: // Use defined ID
            parse_bms_message_0x6B1(frame);
            break;
        default:
            if (DEBUG_MODE) {
                Serial.print("BMSHandler: Received unhandled ID: 0x");
                Serial.println(frame.id, HEX);
            }
            break;
    }

    // TODO: Implement more sophisticated fault checking based on BMS error codes
    const float MIN_CELL_VOLTAGE_EXAMPLE = 2.5f;
    const float MAX_CELL_VOLTAGE_EXAMPLE = 4.2f;
    const int8_t MAX_CELL_TEMP_EXAMPLE = 60;

    current_bms_data.voltage_fault = (current_bms_data.low_cell_voltage < MIN_CELL_VOLTAGE_EXAMPLE || current_bms_data.high_cell_voltage > MAX_CELL_VOLTAGE_EXAMPLE);
    current_bms_data.temperature_fault = (current_bms_data.high_temperature > MAX_CELL_TEMP_EXAMPLE);
}

const BMSData& BMSHandler::get_bms_data() const { return current_bms_data; }

bool BMSHandler::has_critical_fault() const {
    // TODO: Implement ACTUAL critical fault checking logic.
    if (current_bms_data.communication_fault) return true;
    if (current_bms_data.voltage_fault) return true;
    if (current_bms_data.temperature_fault) return true;
    // Add checks for specific general_fault_code values from BMS if critical
    // Example: if (current_bms_data.general_fault_code & SOME_CRITICAL_FAULT_BITMASK) return true;
    // Example: if (! (current_bms_data.relay_state_raw & MAIN_RELAY_CLOSED_BIT) ) return true; // If main relay expected to be closed
    return false;
}

bool BMSHandler::is_communication_active(unsigned long timeout_ms) const {
    if (current_bms_data.last_message_millis == 0 && millis() > timeout_ms) {
        return false;
    }
    return (millis() - current_bms_data.last_message_millis) < timeout_ms;
}

void BMSHandler::parse_bms_message_0x6B0(const CAN_FRAME& frame) {
    if (frame.length == 8) {
        // Pack Current (Bytes 0-1)
        // TODO: Verify byte order (Assuming Big Endian: MSB then LSB) & scaling. Example: 0.1A/bit, signed.
        int16_t raw_current = (static_cast<int16_t>(frame.data.bytes[0] << 8)) | frame.data.bytes[1];
        current_bms_data.pack_current = static_cast<float>(raw_current) * 0.1f;

        // Pack Instantaneous Voltage (Bytes 2-3)
        // TODO: Verify byte order (Assuming Big Endian) & scaling. Example: 0.01V/bit, unsigned.
        uint16_t raw_voltage = (static_cast<uint16_t>(frame.data.bytes[2] << 8)) | frame.data.bytes[3];
        current_bms_data.pack_voltage = static_cast<float>(raw_voltage) * 0.01f;

        // Pack SoC (Byte 4)
        // TODO: Verify scaling. Example: 0.5%/bit.
        current_bms_data.pack_soc = static_cast<float>(frame.data.bytes[4]) * 0.5f;

        // Relay State (Byte 5)
        // TODO: Interpret the bits of this byte based on BMS manual.
        current_bms_data.relay_state_raw = frame.data.bytes[5];

        // Byte 6 ("in use")
        // TODO: Determine what data is in Byte 6 and parse if needed.
        // Example: current_bms_data.byte_6_0x6B0 = frame.data.bytes[6];

        if(DEBUG_MODE >= 2) {
            Serial.print("BMS 0x6B0: I="); Serial.print(current_bms_data.pack_current);
            Serial.print("A, V="); Serial.print(current_bms_data.pack_voltage);
            Serial.print("V, SoC="); Serial.print(current_bms_data.pack_soc);
            Serial.print("%, Relays=0x"); Serial.println(current_bms_data.relay_state_raw, HEX);
        }
    }
}

void BMSHandler::parse_bms_message_0x6B1(const CAN_FRAME& frame) {
    if (frame.length == 8) {
        // Pack DCL (Bytes 0-1)
        // TODO: Verify byte order (Assuming Big Endian) & scaling. Example: 1A/bit, unsigned.
        uint16_t raw_dcl = (static_cast<uint16_t>(frame.data.bytes[0] << 8)) | frame.data.bytes[1];
        current_bms_data.discharge_current_limit = static_cast<float>(raw_dcl);

        // Bytes 2 & 3 are "in use"
        // TODO: Determine what data is in Bytes 2 & 3 and parse if needed.
        // Example: current_bms_data.byte_2_3_0x6B1 = (frame.data.bytes[2] << 8) | frame.data.bytes[3];

        // High Temperature (Byte 4)
        // TODO: Confirm data type (signed/unsigned) and offset. Example: direct degC, signed.
        current_bms_data.high_temperature = static_cast<int8_t>(frame.data.bytes[4]);

        // Low Temperature (Byte 5)
        // TODO: Confirm data type and offset. Example: direct degC, signed.
        current_bms_data.low_temperature = static_cast<int8_t>(frame.data.bytes[5]);

        // Pack CCL (Byte 6)
        // Configured as: 1 byte, Big Endian/MSb first, Multiply by 1, Divide by 1, Add 0, Unsigned.
        // This means raw byte value = Amps.
        current_bms_data.charge_current_limit = static_cast<float>(frame.data.bytes[6]);

        if(DEBUG_MODE >= 2) {
            Serial.print("BMS 0x6B1: DCL="); Serial.print(current_bms_data.discharge_current_limit);
            Serial.print("A, CCL="); Serial.print(current_bms_data.charge_current_limit);
            Serial.print("A, HiTemp="); Serial.print(current_bms_data.high_temperature);
            Serial.print("C, LoTemp="); Serial.println(current_bms_data.low_temperature);
        }
    }
}
