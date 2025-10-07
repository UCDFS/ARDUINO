#pragma once

/* Accelerator Pedal Position Sensor
 See T.11.8 and T.11.9 for more info
Requirments:
- Detect implausibility where there is a 10% difference or more between the two
sensors
- Shut down main motors when implausibility exists for 100ms or more
- Fail safe (shut down main motors) when short to ground or supply, open,
mechanicaly impossible, or otherwise broken
- Fail safe within 500ms of detecting a failure

*/

class APPS {
public:
  APPS();
  APPS(APPS &&) = default;
  APPS(const APPS &) = default;
  APPS &operator=(APPS &&) = default;
  APPS &operator=(const APPS &) = default;
  ~APPS();
  double
  get_apps_reading(); // Returns pedal position (%) or -1.0 on implausibility

private:
  constexpr double PEDAL_MIN;
  constexpr double PEDAL_MAX;
  const int APPS_1_PIN = A6;
  const int APPS_2_PIN = A7;
  const int APPS_BRAKE_PLAUSIBILITY_THRESHOLD =
      25; // % APPS request threshold for brake plausibility check (Rule EV.5.7)

  // APPS
  const float APPS_PLAUSIBILITY_THRESHOLD =
      10.0f; // % difference threshold (Rule EV.5.6)
};

APPS::APPS() {}

APPS::~APPS() {}
