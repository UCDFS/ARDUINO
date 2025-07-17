# UCD Formula Student - EV VCU Software (Arduino Due)

This repository contains the Vehicle Control Unit (VCU) software for the UCD Formula Student electric race car, targeting the Arduino Due platform. The codebase is designed for robust CAN communication, sensor integration, and compliance with FSUK safety rules.

## Features
- **CAN Communication:** Modular CAN interface for Bamocar D3 motor controller and Orion BMS 2, with support for custom message parsing and error handling.
- **Motor Control:** Torque mapping, off-throttle regenerative braking, and battery CCL management for the Emrax 208 motor.
- **Sensor Integration:** Reads and validates APPS (Accelerator Pedal Position Sensors), brake pressure sensors, and MPU6050 accelerometer for deceleration-based logic.
- **Safety Compliance:** Implements APPS plausibility checks, APPS/Brake interlocks, and error pin monitoring for IMD/BSPD faults as per FSUK rules.
- **Dashboard Support:** Nextion display integration for real-time telemetry, lap timing, and system status.
- **Calibration & Debugging:** Extensive debug output, auto-calibration routines, and plottable data for sensor and system validation.

## Directory Structure
```
include/         # Header files for modules and hardware abstraction
lib/             # External libraries (Bamocar, CAN, etc.)
src/             # Main source files (VCU logic, sensor handlers, dashboard, etc.)
test/            # Unit and integration tests
platformio.ini   # PlatformIO build configuration
README.md        # Project documentation
```

## Key Modules
- **main.cpp:** Main VCU loop, CAN updates, motor control, brake light logic, and debug output.
- **apps.cpp:** Reads and validates APPS sensors, implements plausibility checks.
- **brake_light.cpp:** Handles brake pressure averaging, dynamic thresholding, and deceleration-based brake light activation.
- **dashboard.cpp:** Nextion display integration for telemetry and lap timing.
- **monitor_errors.cpp:** Reads digital error pins (IMD, BSPD, etc.) and sets global error flags.
- **simple_can.cpp/h:** Simplified CAN interface for Bamocar and BMS communication.

## Calibration & Setup
- **APPS Calibration:** Update `APPS1_RAW_MIN/MAX` and `APPS2_RAW_MIN/MAX` in `apps.cpp` based on measured pedal sensor values.
- **Brake Light Threshold:** The brake light threshold is dynamically calculated at startup using the initial brake pressure readings. Adjust `BRAKE_THRESHOLD_DELTA` in `brake_light.cpp` for desired sensitivity.
- **Error Pins:** Define and map error pins (22-37) in `header.h` and `monitor_errors.cpp` for IMD/BSPD fault detection.
- **CAN IDs:** Confirm Bamocar and BMS CAN IDs in `simple_can.h` and `bamocar-registers.h` match your hardware configuration.

## Building & Flashing
This project uses PlatformIO for building and uploading firmware:
```sh
platformio run
platformio upload
```

## Outstanding Tasks
- Implement full BMS message parsing and critical fault logic.
- Finalize APPS and brake sensor calibration.
- Verify all CAN message scaling and register mappings.
- Complete error pin mapping and safety interlocks.
- Expand dashboard telemetry and lap timing features.

## Documentation & Support
- For hardware wiring, sensor calibration, and CAN message details, see the comments in each module and the official FSUK rules.
- For team support and collaboration, contact Shane Whelan or the UCD Formula Student team.

---

**Note:** This repository is under active development. For stable releases, refer to the official UCD Formula Student repository: [https://github.com/UCDFS/ARDUINO](https://github.com/UCDFS/ARDUINO)
