/**
 * @file bms_handler.h
 * @brief Defines the BMSHandler class for processing Orion BMS 2 CAN messages
 * and storing the relevant battery status.
 * @author Shane Whelan (UCD Formula Student)
 * @date 2025-04-27
 */

// TODO:
// - Review and update the BMSData struct fields to match ALL data
//   you need from your Orion BMS 2 configuration.
// - Verify scaling and offset for ALL parsed values in bms_handler.cpp.

#ifndef BMS_HANDLER_H
#define BMS_HANDLER_H

#include <stdint.h>
#include <due_can.h> // For CAN_FRAME type

// Structure to hold BMS data
typedef struct {
    // From 0x6B0
    float pack_current;         // Parsed from Byte 0, 1
    float pack_voltage;         // Parsed from Byte 2, 3
    float pack_soc;             // Parsed from Byte 4
    uint8_t relay_state_raw;    // Raw byte for relay state (Byte 5) - parse this further
    // uint8_t byte_6_0x6B0;    // Placeholder for whatever is in Byte 6 of 0x6B0

    // From 0x6B1
    float discharge_current_limit; // Parsed from Byte 0, 1
    float charge_current_limit;    // Parsed from Byte 6 of 0x6B1 (1 byte)
    int8_t high_temperature;       // Parsed from Byte 4
    int8_t low_temperature;        // Parsed from Byte 5
    // uint16_t byte_2_3_0x6B1; // Placeholder if you need to parse data from Bytes 2 & 3 of 0x6B1

    // Cell specific data (often from different CAN IDs or needs to be requested)
    // TODO: Verify if these are received from 0x6B0/0x6B1 or other IDs.
    // If from other IDs, add parsing logic and ensure CANManager filters for them.
    float high_cell_voltage;    // Highest cell voltage (V)
    float low_cell_voltage;     // Lowest cell voltage (V)
    float avg_cell_voltage;     // Average cell voltage (V)
    int8_t avg_temperature;     // Average cell temperature (°C)


    // Other useful derived or status flags
    bool communication_fault;
    bool voltage_fault;         // Overall pack or cell voltage fault
    bool temperature_fault;     // Overall pack or cell temperature fault
    uint16_t general_fault_code; // Example, if BMS provides a general fault code
    unsigned long last_message_millis;
} BMSData;


class BMSHandler {
public:
    BMSHandler();
    void handle_incoming_frame(const CAN_FRAME& frame);
    const BMSData& get_bms_data() const;
    bool has_critical_fault() const;
    bool is_communication_active(unsigned long timeout_ms = 1000) const;

private:
    BMSData current_bms_data;
    void parse_bms_message_0x6B0(const CAN_FRAME& frame);
    void parse_bms_message_0x6B1(const CAN_FRAME& frame);
    // TODO: Add parsing functions for other BMS message IDs if any,
    // especially for individual cell data if not in 0x6B0/0x6B1.
};

extern BMSHandler bms_handler;

#endif // BMS_HANDLER_H
