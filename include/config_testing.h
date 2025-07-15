/**
 * @file config_testing.h
 * @brief Configuration flags for different testing scenarios.
 * @author Shane Whelan (UCD Formula Student)
 * @date 2025-05-22
 */

#ifndef CONFIG_TESTING_H
#define CONFIG_TESTING_H

// BENCH_TEST_MODE:
// Set to 'true' when testing on the bench without BMS, motor controller,
// or other major hardware components connected. This will bypass certain
// hardware checks and communication attempts to prevent errors and allow
// testing of other software functionalities.
//
// Set to 'false' for normal vehicle operation with all hardware connected.
//
// WARNING: Ensure this is FALSE for any on-vehicle testing or operation.
//          Bypassing safety checks can be dangerous.
const bool BENCH_TEST_MODE = false;

#endif // CONFIG_TESTING_H
