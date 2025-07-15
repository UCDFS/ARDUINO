/**
 * @file debug_cycler.cpp
 * @brief Defines and handles runtime cycling of the global DEBUG_MODE variable via a button.
 * @author Shane Whelan (UCD Formula Student)
 * @date 2025-05-23
 */

#include "header.h" // For DEBUG_CYCLE_BUTTON_PIN, Serial, etc.

// --- Global Variable Definitions ---
int DEBUG_MODE = 1; // Definition and initial value for the global debug mode level.
                    // Set to 0 for off, 1 for basic, 3 for most verbose on startup.
const int MAX_DEBUG_LEVEL = 3; // Definition for the maximum debug level for cycling (0-3).

// --- Debug Mode Button Variables ---
static int last_button_state_cycler = HIGH; // Assuming INPUT_PULLUP, so HIGH is not pressed
static unsigned long last_button_press_time_cycler = 0;
const unsigned long DEBOUNCE_DURATION_CYCLER = 50; // ms for debouncing
const unsigned long BUTTON_COOLDOWN_CYCLER = 300;    // ms, minimum time between recognized presses

/**
 * @brief Initializes the debug cycle button pin and prints the initial debug mode.
 */
void setup_debug_cycler() {
    pinMode(DEBUG_CYCLE_BUTTON_PIN, INPUT_PULLUP);
    last_button_state_cycler = digitalRead(DEBUG_CYCLE_BUTTON_PIN); 
    
    // Print initial debug mode only if Serial is likely available and mode is not 0
    if (DEBUG_MODE > 0 && Serial) { 
        Serial.print("Initial DEBUG_MODE set to: ");
        Serial.println(DEBUG_MODE);
    }
}

/**
 * @brief Checks the debug cycle button and updates the global DEBUG_MODE.
 * Call this in the main loop.
 */
void loop_check_debug_cycler() {
    int current_button_state = digitalRead(DEBUG_CYCLE_BUTTON_PIN);
    unsigned long current_time = millis();

    // Debounce logic
    static unsigned long last_state_change_time_cycler = 0;
    static int debouncing_state_cycler = HIGH; 

    if (current_button_state != debouncing_state_cycler) {
        last_state_change_time_cycler = current_time;
        debouncing_state_cycler = current_button_state;
    }

    if ((current_time - last_state_change_time_cycler) > DEBOUNCE_DURATION_CYCLER) {
        // State is considered stable
        if (current_button_state == LOW && last_button_state_cycler == HIGH) { // Falling edge (button pressed)
            // Check cooldown
            if (current_time - last_button_press_time_cycler > BUTTON_COOLDOWN_CYCLER) {
                DEBUG_MODE = (DEBUG_MODE + 1) % (MAX_DEBUG_LEVEL + 1); // Cycles 0-1-2-3-0
                
                if (Serial) { 
                    Serial.print("\n*** DEBUG MODE CHANGED TO: ");
                    Serial.print(DEBUG_MODE);
                    Serial.println(" ***\n");
                }
                last_button_press_time_cycler = current_time; 
            }
        }
        last_button_state_cycler = current_button_state; 
    }
}
