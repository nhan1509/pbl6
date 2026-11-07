// Regression for Arduino .ino preprocessing: generated declarations appear
// before sketch globals/functions, but after the sketch's header includes.
#include <Arduino.h>
#include "../../esp32_line_3sensor_ultrasonic_avoid/RobotTypes.h"

Sensors readRaw();
ClearanceState sonarClearance(uint8_t i);
const char *modeName(Mode m);
void logStateChange(Mode from, Mode to, const char *reason);
void setMode(Mode m, const char *reason);
MotionContext encoderMotionContext();
void updateWheelMonitor(WheelMonitor &wheel, uint32_t count, int target, int actual,
                        MotionContext context, bool insideCurve, float cmPerPulse, uint32_t now);

// Exercise the same implementation and full controller tests with that order.
#include "test.cpp"
