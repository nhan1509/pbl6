#pragma once

#include <Arduino.h>
#include <math.h>

// Keep custom types in a header: Arduino-generated .ino prototypes can be
// inserted before their former inline definitions.

struct Sensors {
  bool l;
  bool c;
  bool r;
};

enum SonarStatus {
  SONAR_WAITING,
  SONAR_OK,
  SONAR_TEMP_LOSS,
  SONAR_LOST,
  SONAR_FAULT
};

enum class EchoMeasurement { UNINITIALIZED, ECHO_VALID, NO_ECHO, SIGNAL_FAULT };

enum class ClearanceState { NEAR, CLEAR, UNKNOWN };

struct SonarFilter {
  float samples[3] = {NAN, NAN, NAN};

  uint8_t count = 0;
  uint8_t index = 0;

  float filtered = NAN;
  float raw = NAN;
  float median = NAN;
  float previousRaw = NAN;

  SonarStatus status = SONAR_WAITING;
  EchoMeasurement measurement = EchoMeasurement::UNINITIALIZED;
  float lastValidRaw = NAN;
  bool nearLatched = false;
  bool cautionLatched = false;
  bool faultLatched = false;
  const char *reason = "UNINITIALIZED";
  uint8_t consecutiveFaults = 0;
  uint32_t lastFaultAt = 0;
  bool pendingHigh = false, stuckHighConfirmed = false;
  uint32_t highSince = 0;

  uint32_t lastGood = 0;
  uint32_t lastMeasurement = 0;
  uint32_t sequence = 0;
  uint32_t validSequence = 0;
  uint32_t validMeasurements = 0;
  uint32_t noEchoMeasurements = 0;
  uint32_t faultMeasurements = 0;
  uint8_t consecutiveValid = 0;
  bool latestValid = false;

  uint8_t consecutiveNoEcho = 0;
};

enum Mode {
  MANUAL,
  FOLLOW_LINE,
  CHECK_SIDES,
  TURN_OUT,
  BYPASS,
  TURN_IN,
  FORWARD_CLEAR,
  FIND_LINE,
  ALIGN_LINE,
  OFFSET_OUT,
  TURN_PARALLEL,
  ALIGN_HEADING,
  ABORTED
};

enum class WheelMotionStatus {
  WHEEL_IDLE, WHEEL_LOW_SPEED, WHEEL_ACCELERATING, WHEEL_MOVING,
  WHEEL_STALL_SUSPECTED, WHEEL_STALL_CONFIRMED
};

enum class MotionContext {
  IDLE, LINE_STRAIGHT, GENTLE_LEFT, GENTLE_RIGHT, SHARP_LEFT, SHARP_RIGHT,
  LINE_RECOVERY, LINE_AMBIGUOUS, AVOID_PRECISE
};

struct WheelMonitor {
  int targetPWM = 0, actualPWM = 0, direction = 0;
  uint32_t lastCount = 0, pulseDelta = 0, lastPulseAt = 0;
  uint32_t requestSince = 0, significantSince = 0, noProgressSince = 0;
  uint32_t suspectedSince = 0, pulseIntervalMs = 0, timeoutMs = 0;
  bool active = false, significant = false, hasPulse = false;
  MotionContext context = MotionContext::IDLE;
  WheelMotionStatus status = WheelMotionStatus::WHEEL_IDLE;
};
