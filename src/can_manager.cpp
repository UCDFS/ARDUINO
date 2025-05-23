/**
 * @file can_manager.cpp
 * @brief Implements the CANManager class for handling CAN bus communication.
 * @author Shane Whelan (UCD Formula Student)
 * @date 2025-04-27
 */

// TODO:
// - Verify and add CAN filters in setup_filters() for ALL required BMS message IDs based on your specific Orion BMS configuration.
// - Confirm Can0.read() behavior in the due_can library regarding reading from multiple filtered mailboxes.

#include "can_manager.h"
#include "bms_handler.h"   // Include BMS handler header
#include "bamocar-due.h" // Include Bamocar header

// Define the global instance
CANManager can_manager;

// Reference to the global Bamocar instance (assuming it's defined elsewhere, e.g., motor_controller.cpp)
extern Bamocar bamocar; // Make sure Bamocar object is accessible

//------------------------------------------------------------------------------
// Constructor
//------------------------------------------------------------------------------
CANManager::CANManager() {
    // Constructor
}

//------------------------------------------------------------------------------
// Initialize CAN Interface and Filters
//------------------------------------------------------------------------------
bool CANManager::initialize(uint32_t baudrate) {
    if (!Can0.begin(baudrate)) {
        if (DEBUG_MODE) {
            Serial.println("CAN0 Initialization Failed!");
        }
        return false;
    }

    if (!setup_filters()) {
         if (DEBUG_MODE) {
            Serial.println("CAN0 Filter Setup Failed!");
        }
        // return false; // Decide if this is critical
    }

    if (DEBUG_MODE) {
        Serial.println("CAN0 Initialized Successfully with Filters.");
    }
    return true;
}

//------------------------------------------------------------------------------
// Configure Hardware Filters
//------------------------------------------------------------------------------
bool CANManager::setup_filters() {
    // Reset all mailboxes/filters to a known state first
    Can0.reset_all_mailbox(); // Corrected function name

    // The due_can library allows up to 8 filters (0-7) for CAN0.
    // Each filter can be assigned to one or more mailboxes implicitly by the library
    // or explicitly if using mailbox-specific filter functions (which are less common in basic usage).
    // We will set up filters for specific IDs.
    // For an exact ID match on a standard 11-bit ID, the mask is 0x7FF.
    const uint32_t EXACT_STD_MASK = 0x7FF;

    // Filter 0: Bamocar TX ID (responses from Bamocar)
    // TODO: Verify BAMOCAR_TX_ID is correct
    if (!Can0.setRXFilter(0, BAMOCAR_TX_ID, EXACT_STD_MASK, false)) { // filter_num, id, mask, extended
        if (DEBUG_MODE) Serial.println("Failed to set filter 0 for Bamocar");
        return false;
    }

    // Filter 1: BMS Message ID 1
    // TODO: Replace ORION_BMS_ID_1 with actual ID
    if (!Can0.setRXFilter(1, ORION_BMS_ID_1, EXACT_STD_MASK, false)) {
        if (DEBUG_MODE) Serial.println("Failed to set filter 1 for BMS_ID_1");
        return false;
    }

    // Filter 2: BMS Message ID 2
    // TODO: Replace ORION_BMS_ID_2 with actual ID
    if (!Can0.setRXFilter(2, ORION_BMS_ID_2, EXACT_STD_MASK, false)) {
        if (DEBUG_MODE) Serial.println("Failed to set filter 2 for BMS_ID_2");
        return false;
    }

    // --- IMPORTANT ---
    // TODO: Add filters here for ALL BMS message IDs you expect to receive.
    // Use subsequent filter numbers (3, 4, ..., up to 7).
    // Example:
    // if (!Can0.setRXFilter(3, YOUR_NEXT_BMS_ID, EXACT_STD_MASK, false)) {
    //     if (DEBUG_MODE) Serial.println("Failed to set filter 3 for YOUR_NEXT_BMS_ID");
    //     return false;
    // }
    // --- --- --- --- ---

    return true;
}


//------------------------------------------------------------------------------
// Process Incoming Messages
//------------------------------------------------------------------------------
void CANManager::process_incoming_messages() {
    CAN_FRAME incoming_frame;

    if (Can0.rx_avail() > 0) { // Use rx_avail() to check for messages
       if(Can0.get_rx_buff(incoming_frame)) { // Use get_rx_buff() to read

            // Dispatch based on CAN ID
            switch (incoming_frame.id) {
                case BAMOCAR_TX_ID:
                    bamocar.handle_incoming_frame(incoming_frame);
                    break;

                // TODO: Add cases for ALL expected BMS IDs here
                case ORION_BMS_ID_1:
                case ORION_BMS_ID_2:
                    bms_handler.handle_incoming_frame(incoming_frame);
                    break;

                default:
                    if (DEBUG_MODE) {
                        Serial.print("CANManager: Received unexpected ID (passed filters): 0x");
                        Serial.println(incoming_frame.id, HEX);
                    }
                    break;
            }
       } else {
            if (DEBUG_MODE) {
                 Serial.println("CANManager: Can0.rx_avail() > 0 but Can0.get_rx_buff() failed.");
            }
       }
    }
}

//------------------------------------------------------------------------------
// Send CAN Message
//------------------------------------------------------------------------------
// Changed parameter to non-const to match due_can library's sendFrame
bool CANManager::send_message(CAN_FRAME& frame) { // Made frame non-const
    return Can0.sendFrame(frame);
}
