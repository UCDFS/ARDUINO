/**
 * @file can_manager.cpp
 * @brief Implements the CANManager class for handling CAN bus communication.
 * @author Shane Whelan (UCD Formula Student)
 * @date 2025-04-27
 */

// TODO:
// - If 'ACCEPT ALL' filter works, revert to specific filters for Bamocar and BMS
//   once the underlying issue with setRXFilter returning unexpected values is resolved.
// - Confirm Can0.get_rx_buff() behavior if issues arise.
// - Investigate why setRXFilter might return values other than 0 or 1.

#include "can_manager.h"
#include "bms_handler.h"
#include "bamocar-due.h" 
// Ensure Arduino.h is included, typically via header.h or other project files

// Define the global instance
CANManager can_manager;

extern Bamocar bamocar; 

CANManager::CANManager() {
    // Constructor
}

bool CANManager::initialize(uint32_t baudrate) {
    if (DEBUG_MODE) Serial.println("CANManager: Attempting Can0.begin()...");
    if (!Can0.begin(baudrate)) {
        if (DEBUG_MODE) {
            Serial.println("CANManager: ERROR - Can0.begin() FAILED!");
        }
        return false;
    }
    if (DEBUG_MODE) Serial.println("CANManager: Can0.begin() SUCCEEDED.");

    if (DEBUG_MODE) Serial.println("CANManager: Attempting setup_filters() in 'ACCEPT ALL' mode...");
    if (!setup_filters()) {
         if (DEBUG_MODE) {
            Serial.println("CANManager: ERROR - setup_filters() FAILED to set 'ACCEPT ALL' filter!");
        }
        return false; 
    } else {
        if (DEBUG_MODE) Serial.println("CANManager: setup_filters() completed ('ACCEPT ALL' mode).");
    }

    if (DEBUG_MODE) {
        Serial.println("CANManager: Initialization complete.");
    }
    return true;
}

bool CANManager::setup_filters() {
    if (DEBUG_MODE) Serial.println("CANManager: Entering setup_filters() for 'ACCEPT ALL' mode.");
    Can0.reset_all_mailbox();
    if (DEBUG_MODE) Serial.println("CANManager: Mailboxes reset.");

    // Configure filter 0 to accept ALL standard CAN IDs
    // ID = 0, Mask = 0 means all bits are "don't care" for matching
    uint32_t accept_all_id = 0x000;
    uint32_t accept_all_mask = 0x000; 

    if (DEBUG_MODE) Serial.println("CANManager: Attempting to set filter 0 to accept all standard IDs (ID=0x000, Mask=0x000)...");
    uint8_t filter_result = Can0.setRXFilter(0, accept_all_id, accept_all_mask, false); // false for standard ID
    
    if (DEBUG_MODE) Serial.print("CANManager: setRXFilter(0) 'ACCEPT ALL' result: ");
    if (DEBUG_MODE) Serial.println((int)filter_result);

    if (filter_result != 1) { // Assuming 1 is the expected success code from due_can documentation
        if (DEBUG_MODE) Serial.println("CANManager: ERROR - Failed to set 'ACCEPT ALL' filter on filter 0.");
        return false; // If we can't even set an accept-all filter, something is more fundamentally wrong.
    }

    if (DEBUG_MODE) Serial.println("CANManager: Filter 0 configured to accept all standard messages.");
    
    // Disable other filters explicitly to be sure only filter 0 is active with accept-all.
    // The due_can library's setRXFilter might implicitly use mailboxes.
    // reset_all_mailbox() should have disabled them, but this is an extra precaution.
    // Note: The due_can library does not have a simple `disable_filter(i)` function.
    // Filters are tied to mailboxes. `reset_all_mailbox()` is the primary way to clear them.
    // If filter 0 is set to accept all, other specific filters are not strictly necessary
    // for receiving all messages, but they wouldn't harm if set to reject or specific IDs.
    // For this test, we rely on filter 0 being the only one effectively open to all.

    return true; 
}

void CANManager::process_incoming_messages() {
    CAN_FRAME incoming_frame;
    if (Can0.rx_avail() > 0) {
       if(Can0.get_rx_buff(incoming_frame)) {
            if (DEBUG_MODE >= 2) { // More verbose debugging for received messages
                Serial.print("CANManager: RX ID: 0x"); Serial.print(incoming_frame.id, HEX);
                Serial.print(" DLC: "); Serial.print(incoming_frame.length);
                Serial.print(" Data: ");
                for (int i = 0; i < incoming_frame.length; i++) {
                    Serial.print(incoming_frame.data.bytes[i], HEX); Serial.print(" ");
                }
                Serial.println();
            }

            switch (incoming_frame.id) {
                case BAMOCAR_TX_ID: // Still check for Bamocar ID if it's on the bus
                    // For BMS-only test, you might comment this out if Bamocar isn't connected.
                    // bamocar.handle_incoming_frame(incoming_frame); 
                    if (DEBUG_MODE) Serial.println("CANManager: Bamocar message (0x181) received and processed (or would be).");
                    break;
                case ORION_BMS_ID_0x6B0:
                case ORION_BMS_ID_0x6B1:
                    bms_handler.handle_incoming_frame(incoming_frame);
                    break;
                default:
                    // With accept-all, you might see other IDs if any other devices are on the bus
                    if (DEBUG_MODE) {
                        Serial.print("CANManager: Received other ID (not BMS/Bamocar): 0x");
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

bool CANManager::send_message(CAN_FRAME& frame) {
    return Can0.sendFrame(frame);
}
