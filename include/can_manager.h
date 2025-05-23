/**
 * @file can_manager.h
 * @brief Defines the CANManager class for handling CAN bus communication
 * using the due_can library on Arduino Due.
 * This class centralizes initialization, filtering, sending, and receiving.
 * @author Shane Whelan (UCD Formula Student)
 * @date 2025-04-27
 */

#ifndef CAN_MANAGER_H
#define CAN_MANAGER_H

#include <due_can.h>
#include "header.h" // Include common headers/constants

// Define expected CAN IDs (Update these based on actual configuration)
#define BAMOCAR_RX_ID   0x201 // Default Bamocar receive ID (we send to this)
#define BAMOCAR_TX_ID   0x181 // Default Bamocar transmit ID (we receive this)
#define ORION_BMS_ID_1  0x420 // Example BMS ID 1 (Needs verification)
#define ORION_BMS_ID_2  0x421 // Example BMS ID 2 (Needs verification)
// Add other necessary CAN IDs here

class CANManager {
public:
    /**
     * @brief Constructor for CANManager.
     */
    CANManager();

    /**
     * @brief Initializes the CAN0 interface and sets up hardware filters.
     * @param baudrate The desired CAN bus speed (e.g., CAN_BPS_500K).
     * @return True if initialization is successful, false otherwise.
     */
    bool initialize(uint32_t baudrate);

    /**
     * @brief Processes incoming CAN messages from the hardware buffer.
     * This should be called frequently in the main loop.
     * It reads messages and dispatches them to appropriate handlers.
     */
    void process_incoming_messages();

    /**
     * @brief Sends a CAN frame onto the CAN0 bus.
     * @param frame The CAN_FRAME object to send. Must be non-const to match due_can library.
     * @return True if the message was queued for sending, false otherwise.
     */
    bool send_message(CAN_FRAME &frame); // Changed from const CAN_FRAME&

private:
    /**
     * @brief Configures the hardware filters for specific CAN IDs.
     * This is called internally by initialize().
     * @return True if filters were set successfully, false otherwise.
     */
    bool setup_filters();

    // Add pointers or references to handlers if needed, e.g.:
    // Bamocar* bamocar_handler;
    // BMSHandler* bms_handler;
};

// Declare a global instance (or manage instantiation differently if preferred)
extern CANManager can_manager;

#endif // CAN_MANAGER_H
