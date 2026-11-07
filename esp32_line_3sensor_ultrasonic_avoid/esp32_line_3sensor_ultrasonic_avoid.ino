
#include <Arduino.h>
#include <math.h>
#include <string.h>
#include "RobotTypes.h"

// =====================================================
// ESP32 CORE 3.x
// 3 LINE SENSORS + 3 HC-SR04 + L298N
// Median + EMA + NO_ECHO handling
// =====================================================

// ---------------- MOTOR ----------------
#define ENA 25
#define IN1 26
#define IN2 27

#define ENB 33
#define IN3 18
#define IN4 19

// ---------------- LINE ----------------
#define LINE_LEFT   16
#define LINE_CENTER 32
#define LINE_RIGHT  17

// ---------------- SINGLE-CHANNEL OPTICAL ENCODERS ----------------
// D0 must be 3.3V-compatible. RISING counts one edge per slot.
const uint8_t ENCODER_LEFT_PIN = 21;
const uint8_t ENCODER_RIGHT_PIN = 4;
// EXAMPLE values: measure these on your chassis before running AUTO.
constexpr float ENCODER_LEFT_PPR = 20.0f; // Pulses per WHEEL revolution, not motor rev.
constexpr float ENCODER_RIGHT_PPR = 20.0f;
constexpr float WHEEL_LEFT_DIAMETER_CM = 6.5f;
constexpr float WHEEL_RIGHT_DIAMETER_CM = 6.5f;
constexpr float WHEEL_TRACK_CM = 13.0f; // 16cm outer width - two half tire widths (3cm each).
constexpr float VEHICLE_OUTER_WIDTH_CM = 16.0f; // Measured OUTER tire edges, NOT wheel track.
constexpr float VEHICLE_LENGTH_CM = 25.0f;
const float TURN_LEFT_SCALE = 1.0f;
const float TURN_RIGHT_SCALE = 1.0f;
const uint32_t ENCODER_DEGLITCH_US = 250;
const uint32_t ENCODER_STALL_MS = 1500;
const int ENCODER_STALL_PWM = 40;
const int ENCODER_STRONG_PWM = 65;
const uint32_t ENCODER_START_GRACE_MS = 300;
const uint32_t ENCODER_STALL_CONFIRM_MS = 200;
const uint32_t ENCODER_FOLLOW_MAX_MS = 3500;
const uint32_t ENCODER_AVOID_MAX_MS = 2500;
const uint32_t ENCODER_NO_MOTION_MS = 2500;
const float ENCODER_REFERENCE_PULSE_CM = 1.021f;
const float ENCODER_SYNC_KP = 5.0f; // PWM per cm of accumulated wheel mismatch.
const int ENCODER_MAX_TRIM = 15;
const float TURN_DEGREES = 90.0f;
const float MIN_BYPASS_CM = 8.0f;
const float MAX_FIND_DISTANCE_CM = 45.0f;
const float MAX_OFFSET_DISTANCE_CM = 80.0f; // Fault limit only; never a successful exit.
const float MAX_BYPASS_DISTANCE_CM = 120.0f;
const float SIDE_SONAR_TO_REAR_CM = VEHICLE_LENGTH_CM; // Side sonars mounted at front.
const uint32_t MOTOR_SETTLE_MS = 200;
const uint32_t MAX_SETTLE_MS = 1500;
const uint8_t EDGE_SAMPLES = 3;

static_assert(ENCODER_LEFT_PPR > 0 && ENCODER_RIGHT_PPR > 0, "PPR must be positive");
static_assert(WHEEL_TRACK_CM > 0 && WHEEL_LEFT_DIAMETER_CM > 0 &&
              WHEEL_RIGHT_DIAMETER_CM > 0, "Measure chassis geometry");

const int BLACK_LEVEL = HIGH;

// ---------------- ULTRASONIC ----------------
// Change these if you have rewired the sensors.
const uint8_t TRIG[3] = {23, 22, 13};
const uint8_t ECHO[3] = {34, 35, 14};

enum {
  FRONT = 0,
  LEFT  = 1,
  RIGHT = 2
};

// ---------------- DISTANCE ----------------
const float OBSTACLE_CM = 10.0f;
const float FRONT_WARNING_CM = 20.0f;
const float FRONT_EMERGENCY_CM = 5.0f;
const float FRONT_CLEAR_CM = 15.0f;
// Measured from the sensor face. Add any body overhang to these offsets.
const float LEFT_BODY_OFFSET_CM = 0.0f;
const float RIGHT_BODY_OFFSET_CM = 0.0f;
const float SIDE_MIN_GAP_CM = 4.0f;
const float SIDE_RESUME_MARGIN_CM = 1.0f;
const float TURN_ENTRY_CM = 20.0f;
const float FRONT_ALERT_RELEASE_CM = 13.0f;
const float FRONT_CAUTION_RELEASE_CM = 23.0f;
const bool ALLOW_UNKNOWN_STARTUP = true;
const bool ALLOW_NO_ECHO_AFTER_FAR = true;
const int UNKNOWN_PROBE_PWM = 60;
const int NO_ECHO_FAR_PWM = 65;

// Maximum distance accepted as a valid echo.
// With a 12ms pulseIn timeout, max theoretical
// measurable distance is about 206cm.
const float MIN_VALID_CM = 2.0f;
const uint32_t ECHO_TIMEOUT_US = 12000;
const float MAX_USAGE_CM = 190.0f;
const float MAX_VALID_CM = ECHO_TIMEOUT_US * 0.0343f / 2.0f;
const uint32_t FRONT_FAULT_PAUSE_MS = 240;
const uint32_t ECHO_HIGH_FAULT_MS = 60;
const uint8_t PERSISTENT_FAULT_SAMPLES = 5;
const int DEGRADED_FOLLOW_PWM = 60;

// Trigger only one HC-SR04 at a time.
// Sequence: FRONT, LEFT, FRONT, RIGHT.
const uint32_t SONAR_GAP_MS = 60;

// Start-to-start: FRONT ~120ms, LEFT/RIGHT ~240ms. Allow scheduling jitter.
const uint32_t SENSOR_STALE_MS = 800;
const uint32_t SONAR_HOLD_MS = 500;
const uint32_t SONAR_FILTER_RESET_MS = 800;
const uint8_t SONAR_FILTER_RESET_LOSSES = 3;
const uint8_t SONAR_RECOVERY_SAMPLES = 2;
static_assert(SONAR_RECOVERY_SAMPLES >= 2, "Clearance recovery requires at least two echoes");
static_assert(SENSOR_STALE_MS > 4 * SONAR_GAP_MS, "Stale must exceed a full side sonar cycle");
const float EMA_REBASE_DELTA_CM = 12.0f;
const float MEDIAN_STABLE_SPREAD_CM = 3.0f;

// Pause immediately on unknown clearance; abort if it cannot be re-confirmed.
const uint32_t CLEARANCE_WAIT_MS = 1500;

// Median and EMA
const float EMA_ALPHA = 0.55f;

// Confirmations based on NEW measurements
const uint8_t FRONT_CONFIRM = 2;
const uint8_t SIDE_CONFIRM = 2;

// ---------------- CONTROL ----------------
const uint32_t CONTROL_MS = 10;
const uint32_t FILTER_MS = 5;

// Time limits are fault guards, never substitutes for encoder completion.
const uint32_t MAX_TURN_OUT_MS = 6000;
const uint32_t MAX_MOVE_MS = 8000;
const uint32_t EDGE_CONFIRM_MS = 350;

const uint32_t MAX_FIND_MS = 8000;
const uint32_t MAX_BYPASS_MS = 12000;
const uint32_t MAX_AVOID_TOTAL_MS = 45000;

// ---------------- SPEED ----------------
int baseSpeed = 70;

const int PWM_STEP_UP = 12;
const int PWM_STEP_DOWN = 45;

const int AVOID_PWM = 85;
const int SEARCH_PWM = 75;
const int SLOW_PWM = 65;
const float LINE_KP = 18.0f;
const float LINE_KD = 8.0f;
const float LINE_ERROR_ALPHA = 0.4f;
const int LINE_MAX_CORRECTION = 30;
const int LINE_CORNER_PWM = 60;
const int LINE_RECOVERY_PWM = 55;
const uint32_t LINE_LOSS_RECOVERY_MS = 350;
const uint32_t LINE_STRAIGHT_GRACE_MS = 80;
const uint8_t LINE_REACQUIRE_CONFIRM = 3;

// =====================================================
// LINE SENSOR STRUCTURES
// =====================================================



Sensors stable = {false, false, false};
Sensors rawPrev = {false, false, false};

uint32_t changedAt[3] = {0, 0, 0};
float lineError = 0, lineDerivative = 0;
int lastLineDirection = 0;
uint32_t lastLineControl = 0, lineLostSince = 0;
bool recoveringLine = false;
uint8_t lineReacquireCount = 0;

void resetLineController() {
  lineError = lineDerivative = 0;
  lastLineDirection = 0;
  recoveringLine = false;
  lineReacquireCount = 0;
  lastLineControl = millis();
}

// =====================================================
// SONAR STATES
// =====================================================








SonarFilter sonar[3];

uint8_t nextSonar = 0;
uint32_t lastSonar = 0;

uint32_t lastFrontProcessed = 0;
uint32_t lastWallProcessed = 0;
uint32_t sideProcessed[2] = {0, 0};
uint8_t sideReadyCount[2] = {0, 0};
uint32_t turnFrontProcessed = 0;
uint8_t turnClearCount = 0;
uint32_t clearanceWaitSince = 0;
bool waitingClearance = false;
bool avoidRight = true;
uint8_t wallSide = LEFT;

uint8_t frontNearCount = 0;
uint8_t seenWallCount = 0;
uint8_t clearWallCount = 0;
uint8_t lineCount = 0;

bool sawWall = false;
bool sawOffsetObject = false;
uint8_t offsetSeenCount = 0, offsetClearCount = 0;
uint32_t offsetProcessed = 0, offsetLastSeenAt = 0;
float obstacleReferenceCm = OBSTACLE_CM;
float offsetEdgeDistance = NAN;
float measuredOffsetCm = 0;

// =====================================================
// MOTOR STATES
// =====================================================

int currentLeft = 0;
int currentRight = 0;
int targetLeftPWM = 0, targetRightPWM = 0;

uint32_t lastControl = 0;
uint32_t stateSince = 0;
uint32_t avoidSince = 0;
uint32_t wallLastSeenAt = 0;

// =====================================================
// ROBOT STATES
// =====================================================



Mode mode = MANUAL;
const char *controlAction = "STOP_MANUAL";

void printSeparator() { Serial.println("--------------------------------------------------"); }
void beginLog(const char *title) { printSeparator(); Serial.printf("[%s]\n", title); }
void endLog() { printSeparator(); }
void logMessage(const char *title, const char *message) {
  beginLog(title); Serial.println(message); endLog();
}

// ISR counters are unsigned path lengths: D0 alone cannot measure direction.
portMUX_TYPE encoderMux = portMUX_INITIALIZER_UNLOCKED;
volatile uint32_t encoderLeftPulses = 0;
volatile uint32_t encoderRightPulses = 0;
volatile uint32_t encoderLeftEdgeUs = 0;
volatile uint32_t encoderRightEdgeUs = 0;
uint32_t phaseLeftStart = 0, phaseRightStart = 0;
uint32_t syncLeftStart = 0, syncRightStart = 0;
int syncLeftTarget = 0, syncRightTarget = 0;
WheelMonitor leftMonitor, rightMonitor;
bool bothMotionArmed = false;
uint32_t bothMotionSince = 0, bothLeftCount = 0, bothRightCount = 0;
bool settlingMotion = false;
uint32_t settleSince = 0, settleQuietSince = 0;
uint32_t settleLeftCount = 0, settleRightCount = 0;

void ARDUINO_ISR_ATTR encoderLeftISR() {
  uint32_t now = micros();
  portENTER_CRITICAL_ISR(&encoderMux);
  if (encoderLeftPulses == 0 || now - encoderLeftEdgeUs >= ENCODER_DEGLITCH_US) {
    encoderLeftPulses++;
    encoderLeftEdgeUs = now;
  }
  portEXIT_CRITICAL_ISR(&encoderMux);
}

void ARDUINO_ISR_ATTR encoderRightISR() {
  uint32_t now = micros();
  portENTER_CRITICAL_ISR(&encoderMux);
  if (encoderRightPulses == 0 || now - encoderRightEdgeUs >= ENCODER_DEGLITCH_US) {
    encoderRightPulses++;
    encoderRightEdgeUs = now;
  }
  portEXIT_CRITICAL_ISR(&encoderMux);
}

void readEncoderCounts(uint32_t &left, uint32_t &right) {
  portENTER_CRITICAL(&encoderMux);
  left = encoderLeftPulses;
  right = encoderRightPulses;
  portEXIT_CRITICAL(&encoderMux);
}

void resetWheelMonitoring() {
  uint32_t l, r; readEncoderCounts(l, r);
  leftMonitor = WheelMonitor(); rightMonitor = WheelMonitor();
  leftMonitor.lastCount = l; rightMonitor.lastCount = r;
  bothMotionArmed = false;
}

float leftPulseCm() {
  return 3.14159265f * WHEEL_LEFT_DIAMETER_CM / ENCODER_LEFT_PPR;
}

float rightPulseCm() {
  return 3.14159265f * WHEEL_RIGHT_DIAMETER_CM / ENCODER_RIGHT_PPR;
}

float phaseLeftCm() {
  uint32_t l, r; readEncoderCounts(l, r);
  return (uint32_t)(l - phaseLeftStart) * leftPulseCm();
}

float phaseRightCm() {
  uint32_t l, r; readEncoderCounts(l, r);
  return (uint32_t)(r - phaseRightStart) * rightPulseCm();
}

// Use the slower wheel so one spinning wheel cannot finish a straight leg.
float phaseDistanceCm() {
  uint32_t l, r; readEncoderCounts(l, r);
  return min((uint32_t)(l - phaseLeftStart) * leftPulseCm(),
             (uint32_t)(r - phaseRightStart) * rightPulseCm());
}

float turnWheelCm() {
  bool rightTurn = mode == TURN_OUT || mode == ALIGN_HEADING ? avoidRight : !avoidRight;
  float scale = rightTurn ? TURN_RIGHT_SCALE : TURN_LEFT_SCALE;
  return 3.14159265f * WHEEL_TRACK_CM * TURN_DEGREES / 360.0f * scale;
}

bool turnReached() {
  return phaseDistanceCm() >= turnWheelCm();
}

void resetEncoderPhase() {
  readEncoderCounts(phaseLeftStart, phaseRightStart);
  syncLeftTarget = syncRightTarget = 0;
  resetWheelMonitoring();
}

// =====================================================
// LINE SENSOR FUNCTIONS
// =====================================================

Sensors readRaw() {
  return {
    digitalRead(LINE_LEFT) == BLACK_LEVEL,
    digitalRead(LINE_CENTER) == BLACK_LEVEL,
    digitalRead(LINE_RIGHT) == BLACK_LEVEL
  };
}

void updateLine() {
  Sensors raw = readRaw();

  bool values[3] = {
    raw.l,
    raw.c,
    raw.r
  };

  bool *previous[3] = {
    &rawPrev.l,
    &rawPrev.c,
    &rawPrev.r
  };

  bool *output[3] = {
    &stable.l,
    &stable.c,
    &stable.r
  };

  uint32_t now = millis();

  for (int i = 0; i < 3; i++) {
    if (values[i] != *previous[i]) {
      *previous[i] = values[i];
      changedAt[i] = now;
    }

    if (now - changedAt[i] >= FILTER_MS) {
      *output[i] = values[i];
    }
  }
}

// =====================================================
// MOTOR FUNCTIONS
// =====================================================

void setMotor(int a, int b, int en, int pwm) {
  pwm = constrain(pwm, -255, 255);

  digitalWrite(a, pwm > 0 ? HIGH : LOW);
  digitalWrite(b, pwm < 0 ? HIGH : LOW);

  ledcWrite(en, abs(pwm));
}

void applyMotorPWM(int left, int right) {
  left = constrain(left, -255, 255);
  right = constrain(right, -255, 255);
  setMotor(IN1, IN2, ENA, left);
  setMotor(IN3, IN4, ENB, right);

  currentLeft = left;
  currentRight = right;
}

void drive(int left, int right) {
  targetLeftPWM = constrain(left, -255, 255);
  targetRightPWM = constrain(right, -255, 255);
  applyMotorPWM(targetLeftPWM, targetRightPWM);
}

void stopMotors() {
  drive(0, 0);
  syncLeftTarget = syncRightTarget = 0;
}

int rampPWM(int now, int target) {
  // Equal magnitude slew for forward/backward. Reverse through zero first.
  if ((now > 0 && target < 0) || (now < 0 && target > 0)) {
    int remaining = max(0, abs(now) - PWM_STEP_DOWN);
    return now > 0 ? remaining : -remaining;
  }
  int magnitude = abs(now), desired = abs(target);
  magnitude = desired > magnitude ? min(magnitude + PWM_STEP_UP, desired) :
                                    max(magnitude - PWM_STEP_DOWN, desired);
  return (target != 0 ? target : now) < 0 ? -magnitude : magnitude;
}
void smoothDrive(int left, int right) {
  targetLeftPWM = constrain(left, -255, 255);
  targetRightPWM = constrain(right, -255, 255);
  left = targetLeftPWM; right = targetRightPWM;
  uint32_t l, r; readEncoderCounts(l, r);
  // Correct equal-speed straight travel and in-place turns only. Unequal
  // targets belong to line/wall steering and must not be cancelled by sync.
  if (left != 0 && abs(left) == abs(right)) {
    if (left != syncLeftTarget || right != syncRightTarget) {
      syncLeftStart = l; syncRightStart = r;
    }
    float error = (uint32_t)(l - syncLeftStart) * leftPulseCm() -
                  (uint32_t)(r - syncRightStart) * rightPulseCm();
    int trim = constrain((int)(error * ENCODER_SYNC_KP),
                         -ENCODER_MAX_TRIM, ENCODER_MAX_TRIM);
    syncLeftTarget = left; syncRightTarget = right;
    left = (left > 0 ? 1 : -1) * constrain(abs(left) - trim, 0, 255);
    right = (right > 0 ? 1 : -1) * constrain(abs(right) + trim, 0, 255);
  } else {
    syncLeftTarget = syncRightTarget = 0;
  }
  applyMotorPWM(
    rampPWM(currentLeft, left),
    rampPWM(currentRight, right)
  );
}

bool encoderMove(float leftCm, float rightCm, int pwm) {
  float leftRemaining = fabsf(leftCm) - phaseLeftCm();
  float rightRemaining = fabsf(rightCm) - phaseRightCm();
  bool leftDone = leftRemaining <= 0;
  bool rightDone = rightRemaining <= 0;
  if (leftDone && rightDone) {
    stopMotors();
    return true;
  }
  // Slow down near the endpoint to reduce coast/overshoot at coarse PPR.
  if (max(leftRemaining, rightRemaining) <= 3.0f) pwm = min(pwm, SLOW_PWM);
  // Stop each completed wheel immediately, while the other catches up.
  int l = leftDone ? 0 : (leftCm >= 0 ? pwm : -pwm);
  int r = rightDone ? 0 : (rightCm >= 0 ? pwm : -pwm);
  smoothDrive(l, r);
  if (leftDone || rightDone) applyMotorPWM(leftDone ? 0 : currentLeft, rightDone ? 0 : currentRight);
  return false;
}

// =====================================================
// SONAR FILTER
// =====================================================

float median3(float a, float b, float c) {
  if (a > b) {
    float t = a; a = b; b = t;
  }

  if (b > c) {
    float t = b; b = c; c = t;
  }

  if (a > b) {
    float t = a; a = b; b = t;
  }

  return b;
}

// History contains valid echoes only. Temporary losses preserve it; prolonged
// losses/faults reinitialize it on the first returning valid sample.
void resetSonarHistory(uint8_t i) {
  SonarFilter &s = sonar[i];
  s.count = s.index = 0;
  s.filtered = s.median = NAN;
  s.previousRaw = NAN;
  for (int j = 0; j < 3; j++) s.samples[j] = NAN;
}

void refreshSonarStatus(uint8_t i) {
  SonarFilter &s = sonar[i];
  if (s.status == SONAR_WAITING || s.status == SONAR_FAULT) return;
  // LOST means the polling stream is stale, NOT repeated ordinary timeouts.
  if (s.sequence != 0 && millis() - s.lastMeasurement > SENSOR_STALE_MS) {
    s.status = SONAR_LOST;
    s.consecutiveValid = 0;
  }
}

void addSonarSample(uint8_t i, unsigned long us, bool signalFault = false, const char *faultReason = nullptr) {
  SonarFilter &s = sonar[i];
  uint32_t now = millis();
  refreshSonarStatus(i);
  bool resetHistory = s.status == SONAR_LOST || s.status == SONAR_FAULT ||
                      (s.faultLatched && s.consecutiveValid == 0) ||
                      (s.consecutiveNoEcho >= SONAR_FILTER_RESET_LOSSES && s.consecutiveValid == 0) ||
                      (s.validMeasurements != 0 && now - s.lastGood > SONAR_FILTER_RESET_MS);
  s.lastMeasurement = now;
  s.sequence++;
  s.latestValid = false;

  float cm = us * 0.0343f / 2.0f;
  if (signalFault || (us != 0 && (!isfinite(cm) || cm < MIN_VALID_CM || cm > MAX_VALID_CM))) {
    s.status = SONAR_FAULT;
    s.measurement = EchoMeasurement::SIGNAL_FAULT;
    s.faultLatched = true;
    s.reason = signalFault ? (faultReason ? faultReason : "SIGNAL_ABNORMAL") :
               (cm < MIN_VALID_CM ? "PULSE_TOO_SHORT" : "PULSE_EXCEEDS_TIMEOUT");
    s.lastFaultAt = now;
    if (s.consecutiveFaults < 255) s.consecutiveFaults++;
    s.faultMeasurements++;
    s.consecutiveValid = 0;
    s.raw = NAN;
    // Preserve old MED/EMA for diagnostics, never for clearance.
    return;
  }
  if (us == 0) {
    s.reason = "TIMEOUT";
    s.measurement = EchoMeasurement::NO_ECHO;
    s.noEchoMeasurements++;
    if (s.consecutiveNoEcho < 255) s.consecutiveNoEcho++;
    s.consecutiveValid = 0;
    s.raw = NAN;
    s.status = SONAR_TEMP_LOSS;
    return;
  }

  if (resetHistory) resetSonarHistory(i);
  s.previousRaw = s.consecutiveValid > 0 ? s.raw : NAN;
  s.raw = cm;
  s.reason = cm > MAX_USAGE_CM ? "OUTSIDE_USAGE_RANGE" : "ECHO_OK";
  s.consecutiveNoEcho = 0;
  s.lastValidRaw = cm;
  s.measurement = EchoMeasurement::ECHO_VALID;
  s.latestValid = true;
  s.lastGood = now;
  s.validSequence++;
  s.validMeasurements++;
  if (s.consecutiveValid < 255) s.consecutiveValid++;

  s.samples[s.index] = cm;
  s.index = (s.index + 1) % 3;
  if (s.count < 3) s.count++;
  s.median = cm;
  if (s.count == 2) s.median = (s.samples[0] + s.samples[1]) / 2.0f;
  else if (s.count == 3) s.median = median3(s.samples[0], s.samples[1], s.samples[2]);

  if (!isfinite(s.filtered)) s.filtered = s.median;
  else {
    s.filtered = EMA_ALPHA * s.median + (1.0f - EMA_ALPHA) * s.filtered;
    // Three mutually consistent valid samples establish a changed distance.
    // Rebase only then, avoiding an old EMA tail without accepting one spike.
    if (s.count == 3) {
      float spread = max(s.samples[0], max(s.samples[1], s.samples[2])) -
                     min(s.samples[0], min(s.samples[1], s.samples[2]));
      if (spread <= MEDIAN_STABLE_SPREAD_CM &&
          fabsf(s.filtered - s.median) > EMA_REBASE_DELTA_CM) s.filtered = s.median;
    }
  }
  if (s.consecutiveValid >= SONAR_RECOVERY_SAMPLES) {
    s.status = SONAR_OK;
    s.consecutiveNoEcho = 0;
    s.faultLatched = false;
    s.consecutiveFaults = 0;
  } else s.status = SONAR_TEMP_LOSS; // Fresh raw, but RECOVERING, not clear yet.

  float nearThreshold = i == FRONT ? OBSTACLE_CM :
                        SIDE_MIN_GAP_CM + (i == LEFT ? LEFT_BODY_OFFSET_CM : RIGHT_BODY_OFFSET_CM);
  float cautionThreshold = i == FRONT ? FRONT_WARNING_CM : nearThreshold;
  if (cm <= nearThreshold) s.nearLatched = true;
  if (cm <= cautionThreshold) s.cautionLatched = true;
  float nearRelease = i == FRONT ? FRONT_ALERT_RELEASE_CM : nearThreshold + SIDE_RESUME_MARGIN_CM;
  float cautionRelease = i == FRONT ? FRONT_CAUTION_RELEASE_CM : nearRelease;
  if (s.consecutiveValid >= SONAR_RECOVERY_SAMPLES && isfinite(s.previousRaw)) {
    if (min(cm, min(s.previousRaw, s.filtered)) >= nearRelease) s.nearLatched = false;
    if (min(cm, min(s.previousRaw, s.filtered)) >= cautionRelease) s.cautionLatched = false;
  }
}

// =====================================================
// SONAR POLLING
// =====================================================

void inspectPendingEcho(uint8_t i) {
  SonarFilter &s = sonar[i];
  if (digitalRead(ECHO[i]) == LOW) {
    s.pendingHigh = s.stuckHighConfirmed = false;
  } else if (s.pendingHigh && millis() - s.highSince >= ECHO_HIGH_FAULT_MS) {
    s.pendingHigh = false;
    s.stuckHighConfirmed = true;
    addSonarSample(i, 0, true, "ECHO_STUCK_HIGH");
  }
}

void pollSonar() {
  uint32_t now = millis();
  for (int j = 0; j < 3; j++) { inspectPendingEcho(j); refreshSonarStatus(j); }

  if (now - lastSonar < SONAR_GAP_MS) {
    return;
  }

  lastSonar = now;

  // FRONT -> LEFT -> FRONT -> RIGHT
  uint8_t i;

  switch (nextSonar) {
    case 0: i = FRONT; break;
    case 1: i = LEFT;  break;
    case 2: i = FRONT; break;
    default: i = RIGHT; break;
  }

  nextSonar = (nextSonar + 1) % 4;

  if (digitalRead(ECHO[i]) == HIGH) {
    // A single HIGH observation is not proof of a stuck pin. Wait for a
    // falling edge over subsequent loop iterations; do not trigger while busy.
    if (sonar[i].stuckHighConfirmed) addSonarSample(i, 0, true, "ECHO_STUCK_HIGH");
    else {
      if (!sonar[i].pendingHigh) { sonar[i].pendingHigh = true; sonar[i].highSince = now; }
      addSonarSample(i, 0);
      sonar[i].reason = "ECHO_BUSY_RETRY";
    }
    return;
  }

  digitalWrite(TRIG[i], LOW);
  delayMicroseconds(3);

  digitalWrite(TRIG[i], HIGH);
  delayMicroseconds(10);

  digitalWrite(TRIG[i], LOW);

  unsigned long us = pulseIn(
    ECHO[i],
    HIGH,
    ECHO_TIMEOUT_US
  );

  // A far echo may still be HIGH at the 12ms timeout. Diagnose it only if
  // HIGH persists, instead of converting every out-of-range pulse to FAULT.
  bool incomplete = us == 0 && digitalRead(ECHO[i]) == HIGH;
  addSonarSample(i, us);
  if (incomplete) {
    sonar[i].pendingHigh = true;
    sonar[i].highSince = millis();
    sonar[i].reason = "TIMEOUT_WAITING_FALL";
  }
}

// =====================================================
// SONAR HELPERS
// =====================================================

bool sonarOk(uint8_t i) {
  refreshSonarStatus(i);
  return sonar[i].status == SONAR_OK && sonar[i].latestValid &&
         sonar[i].consecutiveValid >= SONAR_RECOVERY_SAMPLES &&
         isfinite(sonar[i].raw) && isfinite(sonar[i].filtered) &&
         millis() - sonar[i].lastGood <= SENSOR_STALE_MS;
}

// HELD values are diagnostic only, never usable for a movement decision.
bool sonarUsable(uint8_t i) {
  return sonarOk(i);
}

bool confirmedClear(uint8_t i, float minimumCm) {
  return sonarOk(i) && isfinite(sonar[i].previousRaw) &&
         sonar[i].raw >= minimumCm && sonar[i].previousRaw >= minimumCm &&
         sonar[i].filtered >= minimumCm;
}

bool sonarNear(uint8_t i, float thresholdCm) {
  // Stop on the first new close echo even while RECOVERING. A HELD value
  // cannot fabricate a new near observation, or a new edge/clear observation.
  bool freshEcho = sonar[i].latestValid && isfinite(sonar[i].raw) &&
                   millis() - sonar[i].lastGood <= SENSOR_STALE_MS;
  return freshEcho && (sonar[i].raw <= thresholdCm ||
                      (sonarOk(i) && sonar[i].filtered <= thresholdCm));
}

const char *sonarStatusName(uint8_t i) {
  refreshSonarStatus(i);
  switch (sonar[i].status) {
    case SONAR_WAITING: return "WAITING";
    case SONAR_OK: return "OK";
    case SONAR_TEMP_LOSS: return "TEMP_LOSS";
    case SONAR_LOST: return "LOST";
    case SONAR_FAULT: return "FAULT";
    default: return "UNKNOWN";
  }
}

const char *sonarDataName(uint8_t i) {
  refreshSonarStatus(i);
  if (sonar[i].status == SONAR_LOST) return "STALE";
  if (sonar[i].validMeasurements == 0) return "UNKNOWN";
  uint32_t age = millis() - sonar[i].lastGood;
  if (sonar[i].latestValid && age <= SENSOR_STALE_MS) return "FRESH";
  if (age > SONAR_HOLD_MS || sonar[i].status == SONAR_FAULT) return "STALE";
  return sonar[i].latestValid ? "FRESH" : "HELD";
}

bool measurementFresh(uint8_t i) {
  return sonar[i].measurement != EchoMeasurement::UNINITIALIZED &&
         millis() - sonar[i].lastMeasurement <= SENSOR_STALE_MS;
}

ClearanceState sonarClearance(uint8_t i) {
  float nearThreshold = i == FRONT ? FRONT_WARNING_CM :
                        SIDE_MIN_GAP_CM + (i == LEFT ? LEFT_BODY_OFFSET_CM : RIGHT_BODY_OFFSET_CM);
  float clearThreshold = i == FRONT ? FRONT_CAUTION_RELEASE_CM : nearThreshold + SIDE_RESUME_MARGIN_CM;
  if (sonar[i].latestValid && millis() - sonar[i].lastGood <= SENSOR_STALE_MS &&
      sonar[i].raw <= nearThreshold) return ClearanceState::NEAR;
  if (confirmedClear(i, clearThreshold)) return ClearanceState::CLEAR;
  return ClearanceState::UNKNOWN;
}

const char *measurementName(uint8_t i) {
  switch (sonar[i].measurement) {
    case EchoMeasurement::ECHO_VALID: return "VALID";
    case EchoMeasurement::NO_ECHO: return "NO_ECHO";
    case EchoMeasurement::SIGNAL_FAULT: return "FAULT";
    default: return "UNINITIALIZED";
  }
}

const char *clearanceName(uint8_t i) {
  switch (sonarClearance(i)) {
    case ClearanceState::NEAR: return "NEAR";
    case ClearanceState::CLEAR: return "CLEAR";
    default: return "UNKNOWN";
  }
}

const char *modeName(Mode m);

void logControlAction() {
  static const char *lastPrinted = "";
  static uint32_t lastPrintedAt = 0;
  if (strcmp(lastPrinted, controlAction) != 0 && millis() - lastPrintedAt >= 500) {
    beginLog("CONTROL");
    Serial.printf("MODE: %s\nLINE: %d%d%d\nACTION: %s\nPWM_LEFT: %d\nPWM_RIGHT: %d\nFRONT_DISTANCE: %.1f cm\n",
                  modeName(mode), stable.l, stable.c, stable.r, controlAction,
                  currentLeft, currentRight, sonar[FRONT].raw);
    endLog();
    lastPrinted = controlAction;
    lastPrintedAt = millis();
  }
}

// =====================================================
// STATE MANAGEMENT
// =====================================================

const char *modeName(Mode m) {
  switch (m) {
    case MANUAL: return "MANUAL";
    case FOLLOW_LINE: return "FOLLOW_LINE";
    case CHECK_SIDES: return "CHECK_SIDES";
    case TURN_OUT: return "TURN_OUT";
    case OFFSET_OUT: return "OFFSET_OUT";
    case TURN_PARALLEL: return "TURN_PARALLEL";
    case BYPASS: return "BYPASS";
    case FORWARD_CLEAR: return "FORWARD_CLEAR";
    case TURN_IN: return "TURN_IN";
    case FIND_LINE: return "FIND_LINE";
    case ALIGN_HEADING: return "ALIGN_HEADING";
    case ALIGN_LINE: return "ALIGN_LINE";
    case ABORTED: return "ABORTED";
    default: return "UNKNOWN";
  }
}

void logStateChange(Mode from, Mode to, const char *reason) {
  beginLog("STATE CHANGE");
  Serial.printf("FROM: %s\nTO: %s\nREASON: %s\n", modeName(from), modeName(to), reason);
  endLog();
}

void setMode(Mode m, const char *reason = "STATE_TRANSITION") {
  Mode previous = mode;
  mode = m;
  controlAction = m == MANUAL ? "STOP_MANUAL" :
                  (m == ABORTED ? "STOP_ABORTED" :
                   (m == FOLLOW_LINE ? "FOLLOW_LINE_PENDING" : "AVOID_OBSTACLE"));
  stateSince = millis();
  waitingClearance = false;
  if (m != ABORTED) resetEncoderPhase();
  if (m == FOLLOW_LINE) resetLineController();
  settlingMotion = m != MANUAL && m != ABORTED && m != FOLLOW_LINE && m != CHECK_SIDES;
  if (settlingMotion) {
    stopMotors();
    readEncoderCounts(settleLeftCount, settleRightCount);
    settleSince = settleQuietSince = millis();
  }
  if (m == OFFSET_OUT) {
    sawOffsetObject = false;
    offsetSeenCount = offsetClearCount = 0;
    offsetProcessed = sonar[wallSide].sequence;
    offsetLastSeenAt = millis();
    offsetEdgeDistance = NAN;
  }

  if (previous != mode) logStateChange(previous, mode, reason);
}

void abortAvoid(const char *reason) {
  float failedLeftCm = phaseLeftCm(), failedRightCm = phaseRightCm();
  float failedTurnTargetCm = turnWheelCm();
  Mode failedMode = mode;
  stopMotors();
  setMode(ABORTED, reason);

  beginLog("ABORTED");
  Serial.printf(
    "ABORT: %s | send S then A after inspection\n",
    reason
  );
  for (int i = 0; i < 3; i++) {
    Serial.printf("SONAR[%d]=%s RAW=%.1f FILTER=%.1f cm\n",
                  i, sonarStatusName(i), sonar[i].raw, sonar[i].filtered);
  }
  Serial.printf("ENC PHASE L=%.1f R=%.1f cm | TURN_WHEEL_TARGET=%.1f cm | OFFSET=%.1f cm\n",
                failedLeftCm, failedRightCm, failedTurnTargetCm, measuredOffsetCm);
  Serial.printf("FAILED_STATE=%s\n", modeName(failedMode));
  Serial.println("ACTION: STOP_MOTORS | send S then A after inspection");
  endLog();
}

// =====================================================
// LINE FOLLOWING
// =====================================================

void followLine(int speed, int pwmCap = 255) {
  auto lineDrive = [pwmCap](int left, int right) {
    smoothDrive(constrain(left, 0, pwmCap), constrain(right, 0, pwmCap));
    if (currentLeft > pwmCap || currentRight > pwmCap)
      applyMotorPWM(min(currentLeft, pwmCap), min(currentRight, pwmCap));
  };
  uint32_t now = millis();
  uint8_t pattern = (stable.l ? 4 : 0) | (stable.c ? 2 : 0) | (stable.r ? 1 : 0);
  if (pattern == 0) {
    if (!recoveringLine) { recoveringLine = true; lineLostSince = now; }
    lineReacquireCount = 0;
    uint32_t elapsed = now - lineLostSince;
    if (elapsed >= LINE_LOSS_RECOVERY_MS ||
        (lastLineDirection == 0 && elapsed >= LINE_STRAIGHT_GRACE_MS)) {
      controlAction = "STOP_LINE_LOST";
      stopMotors();
    } else {
      controlAction = "RECOVER_LINE_LIMITED";
      int slow = min(speed, LINE_RECOVERY_PWM);
      int correction = lastLineDirection * min(20, slow / 2);
      // Recovery also obeys the current sonar PWM cap.
      smoothDrive(constrain(slow + correction, 0, min(pwmCap, LINE_RECOVERY_PWM)),
                  constrain(slow - correction, 0, min(pwmCap, LINE_RECOVERY_PWM)));
      applyMotorPWM(constrain(currentLeft, 0, min(pwmCap, LINE_RECOVERY_PWM)),
            constrain(currentRight, 0, min(pwmCap, LINE_RECOVERY_PWM)));
    }
    lastLineControl = now;
    return;
  }
  if (recoveringLine) {
    if (++lineReacquireCount < LINE_REACQUIRE_CONFIRM) {
      controlAction = "WAIT_LINE_REACQUIRE";
      stopMotors();
      return;
    }
    recoveringLine = false;
    lineDerivative = 0;
  }
  float observed = 0;
  switch (pattern) {
    case 4: observed = -2; break; // 100
    case 6: observed = -1; break; // 110
    case 2: observed = 0; break;  // 010
    case 3: observed = 1; break;  // 011
    case 1: observed = 2; break;  // 001
    case 7: observed = lineError * 0.5f; speed = min(speed, LINE_CORNER_PWM); break;
    case 5: observed = (float)lastLineDirection; speed = min(speed, LINE_RECOVERY_PWM); break;
  }
  if (pattern == 4 || pattern == 6) lastLineDirection = -1;
  if (pattern == 1 || pattern == 3) lastLineDirection = 1;
  uint32_t dt = now - lastLineControl;
  float previous = lineError;
  lineError += LINE_ERROR_ALPHA * (observed - lineError);
  float delta = (lineError - previous) * CONTROL_MS / (float)max((uint32_t)CONTROL_MS, dt);
  lineDerivative += 0.3f * (delta - lineDerivative);
  lastLineControl = now;
  if (abs((int)observed) >= 1) speed = min(speed, LINE_CORNER_PWM);
  int correction = constrain((int)(LINE_KP * lineError + LINE_KD * lineDerivative),
                             -LINE_MAX_CORRECTION, LINE_MAX_CORRECTION);
  lineDrive(speed + correction, speed - correction);
}
// =====================================================
// MAIN CONTROL
// =====================================================

float sideOffset(uint8_t side) {
  return side == LEFT ? LEFT_BODY_OFFSET_CM : RIGHT_BODY_OFFSET_CM;
}

bool sidesClear(float extra = 0.0f) {
  return confirmedClear(LEFT, SIDE_MIN_GAP_CM + LEFT_BODY_OFFSET_CM + extra) &&
         confirmedClear(RIGHT, SIDE_MIN_GAP_CM + RIGHT_BODY_OFFSET_CM + extra);
}

bool avoidanceClearance(bool frontSafe) {
  if (!frontSafe) return false;
  switch (mode) {
    case TURN_OUT: case TURN_PARALLEL: case TURN_IN: case ALIGN_HEADING: case ALIGN_LINE:
      return sidesClear(SIDE_RESUME_MARGIN_CM); // Both sides are in the swept footprint.
    default:
      return sonarUsable(wallSide); // Translation: front + obstacle side are required.
  }
}

void resetSideChecks() {
  for (int j = 0; j < 2; j++) {
    sideProcessed[j] = sonar[LEFT + j].sequence;
    sideReadyCount[j] = 0;
  }
}

// Require consecutive NEW valid measurements on BOTH sides.
bool freshSidesReady() {
  for (int j = 0; j < 2; j++) {
    uint8_t i = LEFT + j;
    bool clear = confirmedClear(i, SIDE_MIN_GAP_CM + sideOffset(i) +
                                  SIDE_RESUME_MARGIN_CM);
    if (!clear) sideReadyCount[j] = 0;
    if (sonar[i].sequence != sideProcessed[j]) {
      sideProcessed[j] = sonar[i].sequence;
      if (clear && sideReadyCount[j] < SIDE_CONFIRM) sideReadyCount[j]++;
    }
  }
  return sideReadyCount[0] >= SIDE_CONFIRM && sideReadyCount[1] >= SIDE_CONFIRM;
}

// Losses and faulty/stale measurements pause motion without changing the mode.
// A stopped interval is excluded from motion timers, but not the total timeout.
bool waitForClearance(bool clear, uint32_t now) {
  if (!clear) {
    controlAction = mode == OFFSET_OUT || mode == BYPASS ? "WAIT_EDGE_CONFIRMATION" : "WAIT_CLEARANCE";
    stopMotors();
    if (!waitingClearance) {
      waitingClearance = true;
      clearanceWaitSince = now;
      resetSideChecks();
      turnClearCount = 0;
      clearWallCount = lineCount = 0;
      offsetClearCount = 0;
      offsetEdgeDistance = NAN;
      logMessage("WAIT", "Clearance unknown / recovering; motors stopped");
    }
    if (now - clearanceWaitSince > CLEARANCE_WAIT_MS) {
      abortAvoid("clearance not confirmed (front/left/right)");
    }
    return false;
  }
  if (waitingClearance) {
    // 'clear' already requires recovery and two real raw measurements on
    // the relevant sensors. Do not wait for an unrelated opposite side here.
    if (sawWall) wallLastSeenAt += now - clearanceWaitSince;
    stateSince += now - clearanceWaitSince;
    waitingClearance = false;
  }
  return true;
}

void turnAway(int pwm) {
  float arc = turnWheelCm();
  if (avoidRight) encoderMove(arc, -arc, pwm);
  else encoderMove(-arc, arc, pwm);
}

void turnToward(int pwm) {
  float arc = turnWheelCm();
  if (avoidRight) encoderMove(-arc, arc, pwm);
  else encoderMove(arc, -arc, pwm);
}

float edgeBodyMarginCm() {
  float halfWidth = VEHICLE_OUTER_WIDTH_CM / 2.0f;
  // Conservative 90-degree sweep bound: rear clearance + half width + gap.
  // Does not assume that the wheel axle is at the chassis longitudinal center.
  return SIDE_SONAR_TO_REAR_CM + halfWidth + SIDE_MIN_GAP_CM;
}

bool waitForMotorSettle(uint32_t now) {
  if (!settlingMotion) return true;
  stopMotors();
  uint32_t l, r; readEncoderCounts(l, r);
  if (l != settleLeftCount || r != settleRightCount) {
    settleQuietSince = now;
    settleLeftCount = l; settleRightCount = r;
  }
  if (now - settleQuietSince >= MOTOR_SETTLE_MS) {
    resetEncoderPhase(); // Exclude coast pulses from the NEXT maneuver.
    stateSince = now;
    settlingMotion = false;
    offsetProcessed = sonar[wallSide].sequence;
    lastWallProcessed = sonar[wallSide].sequence;
    turnFrontProcessed = sonar[FRONT].sequence;
    turnClearCount = 0;
    return true;
  }
  if (now - settleSince > MAX_SETTLE_MS) abortAvoid("encoder pulses persist while stopped");
  return false;
}

bool checkTurnEncoderAgreement() {
  float l = phaseLeftCm(), r = phaseRightCm();
  float allowance = max(2.0f * max(leftPulseCm(), rightPulseCm()), 2.0f);
  if (fabsf(l - r) > allowance || max(l, r) > turnWheelCm() + allowance) {
    abortAvoid("turn wheel mismatch: check encoder counts and calibration");
    return false;
  }
  return true;
}

MotionContext encoderMotionContext() {
  if (mode == MANUAL || mode == ABORTED || (targetLeftPWM == 0 && targetRightPWM == 0))
    return MotionContext::IDLE;
  if (mode != FOLLOW_LINE) return MotionContext::AVOID_PRECISE;
  if (recoveringLine) return MotionContext::LINE_RECOVERY;
  uint8_t pattern = (stable.l ? 4 : 0) | (stable.c ? 2 : 0) | (stable.r ? 1 : 0);
  switch (pattern) {
    case 2: return MotionContext::LINE_STRAIGHT;
    case 6: return MotionContext::GENTLE_LEFT;
    case 3: return MotionContext::GENTLE_RIGHT;
    case 4: return MotionContext::SHARP_LEFT;
    case 1: return MotionContext::SHARP_RIGHT;
    default: return MotionContext::LINE_AMBIGUOUS;
  }
}

const char *motionContextName(MotionContext context) {
  switch (context) {
    case MotionContext::LINE_STRAIGHT: return "STRAIGHT";
    case MotionContext::GENTLE_LEFT: return "GENTLE_LEFT";
    case MotionContext::GENTLE_RIGHT: return "GENTLE_RIGHT";
    case MotionContext::SHARP_LEFT: return "SHARP_LEFT";
    case MotionContext::SHARP_RIGHT: return "SHARP_RIGHT";
    case MotionContext::LINE_RECOVERY: return "LINE_RECOVERY";
    case MotionContext::LINE_AMBIGUOUS: return "LINE_AMBIGUOUS";
    case MotionContext::AVOID_PRECISE: return "AVOID_PRECISE";
    default: return "IDLE";
  }
}

const char *wheelStatusName(WheelMotionStatus status) {
  switch (status) {
    case WheelMotionStatus::WHEEL_LOW_SPEED: return "LOW_SPEED";
    case WheelMotionStatus::WHEEL_ACCELERATING: return "ACCELERATING";
    case WheelMotionStatus::WHEEL_MOVING: return "MOVING";
    case WheelMotionStatus::WHEEL_STALL_SUSPECTED: return "STALL_SUSPECTED";
    case WheelMotionStatus::WHEEL_STALL_CONFIRMED: return "STALL_CONFIRMED";
    default: return "INTENTIONALLY_STOPPED";
  }
}

void updateWheelMonitor(WheelMonitor &wheel, uint32_t count, int target, int actual,
                        MotionContext context, bool insideCurve, float cmPerPulse, uint32_t now) {
  int oldTarget = wheel.targetPWM;
  MotionContext oldContext = wheel.context;
  wheel.pulseDelta = (uint32_t)(count - wheel.lastCount);
  wheel.lastCount = count;
  wheel.targetPWM = target; wheel.actualPWM = actual; wheel.context = context;
  if (wheel.pulseDelta) {
    if (wheel.hasPulse) {
      uint32_t interval = (now - wheel.lastPulseAt) / wheel.pulseDelta;
      wheel.pulseIntervalMs = wheel.pulseIntervalMs ? (wheel.pulseIntervalMs + interval) / 2 : interval;
    }
    wheel.lastPulseAt = now;
    wheel.hasPulse = true;
    wheel.noProgressSince = now;
  }
  if (target == 0 || context == MotionContext::IDLE) {
    wheel.active = wheel.significant = false;
    wheel.direction = 0;
    wheel.timeoutMs = 0;
    wheel.status = WheelMotionStatus::WHEEL_IDLE;
    return;
  }
  int direction = target > 0 ? 1 : -1;
  bool restart = !wheel.active || direction != wheel.direction ||
                 (context == MotionContext::LINE_STRAIGHT && oldContext != context);
  if (restart) {
    wheel.requestSince = wheel.noProgressSince = now;
    wheel.significant = false;
    wheel.pulseIntervalMs = 0;
    wheel.hasPulse = false;
  }
  wheel.active = true; wheel.direction = direction;

  // Inside-wheel motion below the strong-demand threshold is intentionally
  // uncertain at ~1cm/pulse. Outer-wheel and whole-car checks remain enabled.
  bool lowDemand = abs(target) < ENCODER_STALL_PWM || abs(actual) < ENCODER_STALL_PWM ||
                   (insideCurve && min(abs(target), abs(actual)) < ENCODER_STRONG_PWM);
  bool reversing = actual != 0 && (actual > 0 ? 1 : -1) != direction;
  if (lowDemand || reversing) {
    wheel.significant = false;
    wheel.noProgressSince = now;
    wheel.timeoutMs = 0;
    bool startingRamp = !insideCurve && abs(target) >= ENCODER_STALL_PWM && abs(actual) < ENCODER_STALL_PWM;
    wheel.status = reversing || startingRamp ? WheelMotionStatus::WHEEL_ACCELERATING : WheelMotionStatus::WHEEL_LOW_SPEED;
    return;
  }
  // A material increase in demand gets a new acceleration window; tiny PD
  // updates do not perpetually reset the watchdog.
  if (!wheel.significant || abs(target) - abs(oldTarget) >= 20) {
    wheel.significantSince = wheel.noProgressSince = now;
  }
  wheel.significant = true;
  float resolutionScale = constrain(cmPerPulse / ENCODER_REFERENCE_PULSE_CM, 0.5f, 2.0f);
  int effectivePWM = max(ENCODER_STALL_PWM, min(abs(target), abs(actual)));
  float referencePWM = context == MotionContext::AVOID_PRECISE ? (float)AVOID_PWM : 70.0f;
  uint32_t timeout = (uint32_t)(ENCODER_STALL_MS * resolutionScale * referencePWM / effectivePWM);
  if (wheel.pulseIntervalMs) timeout = max(timeout, min(wheel.pulseIntervalMs, (uint32_t)1200) * 3U);
  uint32_t maximum = context == MotionContext::AVOID_PRECISE ? ENCODER_AVOID_MAX_MS : ENCODER_FOLLOW_MAX_MS;
  wheel.timeoutMs = constrain(timeout, (uint32_t)1000, maximum);
  if (now - wheel.significantSince < ENCODER_START_GRACE_MS) {
    wheel.status = WheelMotionStatus::WHEEL_ACCELERATING;
    return;
  }
  if (now - wheel.noProgressSince < wheel.timeoutMs) {
    wheel.status = WheelMotionStatus::WHEEL_MOVING;
    return;
  }
  if (wheel.status != WheelMotionStatus::WHEEL_STALL_SUSPECTED &&
      wheel.status != WheelMotionStatus::WHEEL_STALL_CONFIRMED) wheel.suspectedSince = now;
  wheel.status = now - wheel.suspectedSince >= ENCODER_STALL_CONFIRM_MS ?
                   WheelMotionStatus::WHEEL_STALL_CONFIRMED : WheelMotionStatus::WHEEL_STALL_SUSPECTED;
}

void printWheelMonitor(const char *name, const WheelMonitor &wheel, int target, int actual) {
  Serial.printf("%s:\n  TARGET_PWM: %d\n  ACTUAL_PWM: %d\n  PULSE_DELTA: %lu\n  STATUS: %s\n"
                "  REQUEST_MS: %lu\n  NO_PROGRESS_MS: %lu\n  TIMEOUT_MS: %lu\n",
                name, target, actual, (unsigned long)wheel.pulseDelta, wheelStatusName(wheel.status),
                wheel.active ? (unsigned long)(millis() - wheel.requestSince) : 0UL,
                wheel.significant ? (unsigned long)(millis() - wheel.noProgressSince) : 0UL,
                (unsigned long)wheel.timeoutMs);
}

void printEncoderMonitoring() {
  Serial.printf("MODE: %s\nLINE: %d%d%d\nMOTION: %s\n", modeName(mode), stable.l, stable.c, stable.r,
                motionContextName(encoderMotionContext()));
  printWheelMonitor("LEFT", leftMonitor, targetLeftPWM, currentLeft);
  printWheelMonitor("RIGHT", rightMonitor, targetRightPWM, currentRight);
  Serial.printf("ACTION: %s\n", controlAction);
}

void logEncoderMonitorChanges() {
  static WheelMotionStatus leftPrinted = WheelMotionStatus::WHEEL_IDLE;
  static WheelMotionStatus rightPrinted = WheelMotionStatus::WHEEL_IDLE;
  static MotionContext contextPrinted = MotionContext::IDLE;
  static uint32_t lastPrintedAt = 0;
  MotionContext context = encoderMotionContext();
  if (mode == MANUAL || mode == ABORTED) return;
  if ((leftPrinted != leftMonitor.status || rightPrinted != rightMonitor.status || contextPrinted != context) &&
      millis() - lastPrintedAt >= 500) {
    beginLog("ENCODER MONITOR"); printEncoderMonitoring(); endLog();
    leftPrinted = leftMonitor.status; rightPrinted = rightMonitor.status;
    contextPrinted = context; lastPrintedAt = millis();
  }
}

bool checkEncoderStall(uint32_t now) {
  // Called AFTER updateControl(): target and actual PWM describe this tick's
  // intent, including zero commands from WAIT/settling/obstacle/line stops.
  if (mode == MANUAL || mode == ABORTED) return true;
  uint32_t l, r; readEncoderCounts(l, r);
  MotionContext context = encoderMotionContext();
  bool insideLeft = context == MotionContext::GENTLE_LEFT || context == MotionContext::SHARP_LEFT;
  bool insideRight = context == MotionContext::GENTLE_RIGHT || context == MotionContext::SHARP_RIGHT;
  updateWheelMonitor(leftMonitor, l, targetLeftPWM, currentLeft, context, insideLeft, leftPulseCm(), now);
  updateWheelMonitor(rightMonitor, r, targetRightPWM, currentRight, context, insideRight, rightPulseCm(), now);

  // Independent of line-pattern changes: both materially driven wheels cannot
  // evade the no-motion check by alternating steering contexts.
  bool bothDriven = targetLeftPWM != 0 && targetRightPWM != 0 &&
                    abs(currentLeft) >= ENCODER_STALL_PWM && abs(currentRight) >= ENCODER_STALL_PWM;
  if (!bothDriven) bothMotionArmed = false;
  else {
    if (!bothMotionArmed || l != bothLeftCount || r != bothRightCount) bothMotionSince = now;
    bothMotionArmed = true;
  }
  bothLeftCount = l; bothRightCount = r;
  bool stationary = bothMotionArmed && now - bothMotionSince >= ENCODER_NO_MOTION_MS;
  const char *reason = stationary ? "VEHICLE_NO_MOTION" :
                       (leftMonitor.status == WheelMotionStatus::WHEEL_STALL_CONFIRMED ? "LEFT_WHEEL_STALL" :
                        (rightMonitor.status == WheelMotionStatus::WHEEL_STALL_CONFIRMED ? "RIGHT_WHEEL_STALL" : nullptr));
  if (!reason) return true;
  if (stationary) leftMonitor.status = rightMonitor.status = WheelMotionStatus::WHEEL_STALL_CONFIRMED;
  beginLog("ENCODER STALL"); printEncoderMonitoring();
  Serial.printf("ACTION: STOP_MOTORS\nREASON: %s\n", reason); endLog();
  abortAvoid(reason);
  return false;
}
void printEncoders() {
  uint32_t l, r; readEncoderCounts(l, r);
  Serial.printf("ENC L=%lu R=%lu | PATH L=%.1f R=%.1f cm | PHASE L=%.1f R=%.1f cm\n",
                (unsigned long)l, (unsigned long)r, l * leftPulseCm(), r * rightPulseCm(),
                phaseLeftCm(), phaseRightCm());
  printEncoderMonitoring();
}

void updateControl() {
  if (mode == MANUAL || mode == ABORTED) return;
  controlAction = mode == FOLLOW_LINE ? "FOLLOW_LINE" : "AVOID_OBSTACLE";
  uint32_t now = millis();


  // Both side guards also apply during normal automatic line following.
  if (sonarNear(LEFT, SIDE_MIN_GAP_CM + LEFT_BODY_OFFSET_CM) ||
      sonarNear(RIGHT, SIDE_MIN_GAP_CM + RIGHT_BODY_OFFSET_CM)) {
    abortAvoid("side gap at/below 4cm (body offset included)");
    return;
  }
  if (sonarNear(FRONT, FRONT_EMERGENCY_CM)) {
    abortAvoid("front emergency distance");
    return;
  }
  if (mode != FOLLOW_LINE && now - avoidSince > MAX_AVOID_TOTAL_MS) {
    abortAvoid("avoid total timeout");
    return;
  }

  if (mode == FOLLOW_LINE) {
    bool degraded = sonar[LEFT].faultLatched || sonar[RIGHT].faultLatched || sonar[FRONT].faultLatched;
    if (sonar[FRONT].validSequence != lastFrontProcessed) {
      lastFrontProcessed = sonar[FRONT].validSequence;
      if (sonar[FRONT].latestValid && sonar[FRONT].raw <= OBSTACLE_CM) {
        if (frontNearCount < FRONT_CONFIRM) frontNearCount++;
      } else if (confirmedClear(FRONT, FRONT_ALERT_RELEASE_CM)) frontNearCount = 0;
      else if (frontNearCount > 1) frontNearCount = 1;
    }
    // Stop on the first near sample while waiting for confirmation.
    if (frontNearCount >= FRONT_CONFIRM && sonarOk(FRONT)) {
      stopMotors();
      beginLog("OBSTACLE");
      Serial.printf("FRONT_DISTANCE: %.1f cm\nACTION: STOP_AND_CHECK_SIDES\n", sonar[FRONT].raw);
      endLog();
      avoidSince = now;
      obstacleReferenceCm = min(sonar[FRONT].raw, sonar[FRONT].filtered);
      frontNearCount = 0;
      sawWall = false;
      seenWallCount = clearWallCount = lineCount = 0;
      resetSideChecks();
      setMode(CHECK_SIDES, "OBSTACLE_CONFIRMED");
      return;
    }
    if (frontNearCount > 0 || sonar[FRONT].nearLatched ||
        sonar[LEFT].nearLatched || sonar[RIGHT].nearLatched) {
      stopMotors();
      waitingClearance = false;
      controlAction = frontNearCount > 0 || sonar[FRONT].nearLatched ?
                        "STOP_OBSTACLE" : "STOP_SIDE_ALERT";
      return;
    }

    // FOLLOW_LINE is a different risk policy from avoidance: side UNKNOWN
    // alone never stops the car, but real side hazards/faults still do.
    waitingClearance = false;
    if (sonar[FRONT].faultLatched &&
        (now - sonar[FRONT].lastFaultAt < FRONT_FAULT_PAUSE_MS ||
         sonar[FRONT].consecutiveFaults >= PERSISTENT_FAULT_SAMPLES)) {
      stopMotors();
      controlAction = "STOP_FRONT_FAULT_RETRY";
      return;
    }
    if (!measurementFresh(FRONT)) {
      stopMotors();
      controlAction = "STOP_FRONT_STALE_OR_UNINITIALIZED";
      return;
    }
    if (sonarOk(FRONT)) {
      bool warning = sonarNear(FRONT, FRONT_WARNING_CM);
      int cap = warning ? SLOW_PWM : 255;
      if (degraded) cap = min(cap, DEGRADED_FOLLOW_PWM);
      controlAction = warning ? "SLOW_FRONT_WARNING" :
                        (degraded ? "FOLLOW_LINE_SIDE_DEGRADED" : "FOLLOW_LINE");
      followLine(min(baseSpeed, cap), cap);
      return;
    }
    if (!sonar[FRONT].latestValid && sonar[FRONT].cautionLatched) {
      stopMotors();
      controlAction = "STOP_LAST_NEAR_HOLD";
      return;
    }
    bool hasHistory = sonar[FRONT].validMeasurements != 0;
    bool allowUnknown = hasHistory ? ALLOW_NO_ECHO_AFTER_FAR : ALLOW_UNKNOWN_STARTUP;
    if (!allowUnknown) {
      stopMotors();
      controlAction = "STOP_UNKNOWN_POLICY_DISABLED";
      return;
    }
    int cap = hasHistory && sonar[FRONT].measurement == EchoMeasurement::NO_ECHO ?
                NO_ECHO_FAR_PWM : UNKNOWN_PROBE_PWM;
    if (degraded) cap = min(cap, DEGRADED_FOLLOW_PWM);
    controlAction = hasHistory ? "FOLLOW_LINE_LIMITED" : "FOLLOW_LINE_PROBE_UNKNOWN";
    followLine(min(baseSpeed, cap), cap);
    return;
  }

  if (mode == CHECK_SIDES) {
    stopMotors();
    if (freshSidesReady() && sonarUsable(FRONT)) {
      bool leftRoom = confirmedClear(LEFT, TURN_ENTRY_CM + LEFT_BODY_OFFSET_CM);
      bool rightRoom = confirmedClear(RIGHT, TURN_ENTRY_CM + RIGHT_BODY_OFFSET_CM);
      if (leftRoom || rightRoom) {
        float leftGap = min(sonar[LEFT].raw, sonar[LEFT].filtered) - LEFT_BODY_OFFSET_CM;
        float rightGap = min(sonar[RIGHT].raw, sonar[RIGHT].filtered) - RIGHT_BODY_OFFSET_CM;
        avoidRight = rightRoom && (!leftRoom || rightGap + 2.0f >= leftGap);
        wallSide = avoidRight ? LEFT : RIGHT;
        turnClearCount = 0;
        turnFrontProcessed = sonar[FRONT].sequence;
        lastWallProcessed = sonar[wallSide].sequence;
        beginLog("AVOID");
        Serial.printf("AVOID=%s WALL=%s L=%.1f R=%.1f cm\n",
                      avoidRight ? "RIGHT" : "LEFT",
                      wallSide == LEFT ? "LEFT" : "RIGHT", leftGap, rightGap);
        endLog();
        // Keep valid filter history. Front clearance still needs new echoes.
        setMode(TURN_OUT, "TURN_SIDE_CONFIRMED");
        return;
      }
    }
    if (now - stateSince > CLEARANCE_WAIT_MS)
      abortAvoid("both sides need valid gaps; chosen side needs turning room");
    return;
  }

  // Initial turn must tolerate the known obstacle still in the front beam.
  // Fresh measurements and the emergency threshold remain mandatory throughout.
  if (mode != TURN_OUT && sonarNear(FRONT, OBSTACLE_CM)) {
    abortAvoid("front blocked during bypass");
    return;
  }
  bool frontSafe = mode == TURN_OUT ? sonarUsable(FRONT) : confirmedClear(FRONT, OBSTACLE_CM);
  if (!waitForClearance(avoidanceClearance(frontSafe), now)) return;
  if (!waitForMotorSettle(now)) return;
  if ((mode == TURN_OUT || mode == TURN_PARALLEL || mode == TURN_IN || mode == ALIGN_HEADING) &&
      !checkTurnEncoderAgreement()) return;

  switch (mode) {
    case TURN_OUT:
      turnAway(AVOID_PWM);
      if (sonar[FRONT].sequence != turnFrontProcessed) {
        turnFrontProcessed = sonar[FRONT].sequence;
        if (turnReached() && confirmedClear(FRONT, FRONT_CLEAR_CM)) {
          if (turnClearCount < FRONT_CONFIRM) turnClearCount++;
        } else turnClearCount = 0;
      }
      // Hold at the measured angle while waiting for fresh front clearance.
      if (turnReached() && turnClearCount >= FRONT_CONFIRM) {
        stopMotors();
        logMessage("AVOID", "TURN_OUT complete: front clear; moving sideways");
        setMode(OFFSET_OUT, "TURN_OUT_COMPLETE");
      } else if (now - stateSince > MAX_TURN_OUT_MS) {
        abortAvoid("front not clear after turn-out");
      }
      break;

    case OFFSET_OUT:
      // Facing sideways: the sonar toward the obstacle scans its front face.
      // It must first see that face, then see a sustained opening. Initial
      // NO_ECHO alone never proves that the body has passed the obstacle edge.
      if (sonar[wallSide].sequence != offsetProcessed) {
        offsetProcessed = sonar[wallSide].sequence;
        // Rotation moves a front-mounted sonar behind its former front plane.
        // Include chassis length so a real front face is not rejected merely
        // because the encoder axle is near the rear of this 25cm chassis.
        float faceLimit = obstacleReferenceCm + VEHICLE_LENGTH_CM + VEHICLE_OUTER_WIDTH_CM / 2.0f;
        if (sonarNear(wallSide, faceLimit + sideOffset(wallSide))) {
          offsetLastSeenAt = now;
          if (offsetSeenCount < SIDE_CONFIRM) offsetSeenCount++;
          if (offsetSeenCount >= SIDE_CONFIRM) sawOffsetObject = true;
          offsetClearCount = 0;
          offsetEdgeDistance = NAN;
        } else {
          offsetSeenCount = 0;
          if (sawOffsetObject && confirmedClear(wallSide, faceLimit + SIDE_MIN_GAP_CM + sideOffset(wallSide))) {
            if (offsetClearCount == 0) offsetEdgeDistance = phaseDistanceCm();
            if (offsetClearCount < EDGE_SAMPLES) offsetClearCount++;
          } else {
            offsetClearCount = 0;
            offsetEdgeDistance = NAN;
          }
        }
      }
      if (sawOffsetObject && offsetClearCount >= EDGE_SAMPLES &&
          now - offsetLastSeenAt >= EDGE_CONFIRM_MS && isfinite(offsetEdgeDistance) &&
          phaseDistanceCm() - offsetEdgeDistance >= edgeBodyMarginCm()) {
        stopMotors();
        measuredOffsetCm = phaseDistanceCm();
        beginLog("AVOID");
        Serial.printf("OFFSET sonar edge confirmed: moved %.1f cm, body margin %.1f cm\n",
                      measuredOffsetCm, edgeBodyMarginCm());
        endLog();
        setMode(TURN_PARALLEL, "OFFSET_EDGE_AND_BODY_CLEAR");
      } else if (now - stateSince > MAX_MOVE_MS || phaseDistanceCm() > MAX_OFFSET_DISTANCE_CM) {
        abortAvoid("offset: sonar edge not confirmed within limits");
      } else smoothDrive(SLOW_PWM, SLOW_PWM);
      break;

    case TURN_PARALLEL:
      turnToward(AVOID_PWM);
      if (turnReached()) {
        stopMotors();
        sawWall = false;
        seenWallCount = clearWallCount = 0;
        lastWallProcessed = sonar[wallSide].sequence;
        logMessage("AVOID", "BYPASS: straight encoder control; sonar confirms obstacle end");
        setMode(BYPASS, "PARALLEL_TURN_COMPLETE");
      } else if (now - stateSince > MAX_TURN_OUT_MS) abortAvoid("parallel turn timeout");
      break;

    case BYPASS: {
      if (sonar[wallSide].sequence != lastWallProcessed) {
        lastWallProcessed = sonar[wallSide].sequence;
        if (sonarNear(wallSide, edgeBodyMarginCm() + sideOffset(wallSide))) {
          wallLastSeenAt = now;
          if (seenWallCount < SIDE_CONFIRM) seenWallCount++;
          clearWallCount = 0;
          if (seenWallCount >= SIDE_CONFIRM) sawWall = true;
        } else {
          seenWallCount = 0;
          if (confirmedClear(wallSide, edgeBodyMarginCm() + SIDE_MIN_GAP_CM + sideOffset(wallSide))) {
            if (clearWallCount < EDGE_SAMPLES) clearWallCount++;
          } else clearWallCount = 0;
        }
      }
      bool edgePassed = sawWall && phaseDistanceCm() >= MIN_BYPASS_CM &&
                        now - wallLastSeenAt >= EDGE_CONFIRM_MS;
      if (clearWallCount >= EDGE_SAMPLES && edgePassed) {
        stopMotors();
        logMessage("AVOID", "BYPASS complete: sonar side edge passed");
        setMode(FORWARD_CLEAR, "SONAR_EDGE_CONFIRMED"); // Clear rear of chassis BEFORE turning inward.
        break;
      }
      if (now - stateSince > MAX_BYPASS_MS || phaseDistanceCm() > MAX_BYPASS_DISTANCE_CM) {
        abortAvoid("obstacle edge not confirmed");
        break;
      }
      smoothDrive(AVOID_PWM, AVOID_PWM); // Option A: never steer toward the wall.
      break;
    }

    case FORWARD_CLEAR:
      if (encoderMove(edgeBodyMarginCm(), edgeBodyMarginCm(), SLOW_PWM)) {
        stopMotors();
        setMode(TURN_IN, "REAR_CLEARANCE_COMPLETE");
      } else if (now - stateSince > MAX_MOVE_MS) abortAvoid("rear clearance distance timeout");
      break;

    case TURN_IN:
      turnToward(AVOID_PWM);
      if (turnReached()) {
        stopMotors();
        lineCount = 0;
        setMode(FIND_LINE, "RETURN_TURN_COMPLETE");
      } else if (now - stateSince > MAX_TURN_OUT_MS) abortAvoid("return turn timeout");
      break;

    case FIND_LINE:
      // Translate toward the original line; pivoting alone cannot cross a gap.
      smoothDrive(SEARCH_PWM, SEARCH_PWM);
      if (stable.l || stable.c || stable.r) {
        stopMotors();
        if (++lineCount >= LINE_REACQUIRE_CONFIRM) {
          lineCount = 0;
          // Restore travel heading only after repeated debounced line evidence.
          setMode(ALIGN_HEADING, "LINE_REACQUIRED");
        }
      } else lineCount = 0;
      if (mode == FIND_LINE && (now - stateSince > MAX_FIND_MS ||
                 phaseDistanceCm() > max(MAX_FIND_DISTANCE_CM,
                                         measuredOffsetCm + VEHICLE_OUTER_WIDTH_CM + SIDE_MIN_GAP_CM)))
        abortAvoid("line not found within distance/time limit");
      break;

    case ALIGN_HEADING:
      turnAway(AVOID_PWM);
      if (turnReached()) {
        stopMotors();
        lineCount = 0;
        setMode(ALIGN_LINE, "HEADING_RESTORED");
      } else if (now - stateSince > MAX_TURN_OUT_MS) abortAvoid("heading turn timeout");
      break;

    case ALIGN_LINE:
      if (stable.c && !stable.l && !stable.r) {
        stopMotors();
        if (++lineCount >= 3) {
          frontNearCount = 0;
          lastFrontProcessed = sonar[FRONT].validSequence;
          setMode(FOLLOW_LINE, "LINE_ALIGNED");
        }
      } else {
        lineCount = 0;
        if (stable.l && !stable.r) smoothDrive(0, SEARCH_PWM);
        else if (stable.r && !stable.l) smoothDrive(SEARCH_PWM, 0);
        // Local alignment is not a new measured 90-degree maneuver.
        else if (avoidRight) smoothDrive(-SEARCH_PWM, SEARCH_PWM);
        else smoothDrive(SEARCH_PWM, -SEARCH_PWM);
      }
      if (mode == ALIGN_LINE && now - stateSince > MAX_FIND_MS)
        abortAvoid("line alignment timeout");
      break;

    default:
      break;
  }
}

// Keep the public control() entry point for loop/tests. Monitoring follows ALL
// early-return branches from updateControl, so a newly requested stop wins.
void control() {
  updateControl();
  if (mode != MANUAL && mode != ABORTED) {
    checkEncoderStall(millis());
    logEncoderMonitorChanges();
  }
}
// =====================================================
// SERIAL COMMANDS
// =====================================================

bool isNumber(const String &s) {
  if (!s.length()) {
    return false;
  }

  for (unsigned i = 0; i < s.length(); i++) {
    if (!isDigit(s[i])) {
      return false;
    }
  }

  return true;
}

void printSonar(uint8_t i, const char *name) {
  const SonarFilter &s = sonar[i];
  const char *status = sonarStatusName(i);
  float noEchoRate = s.sequence ? 100.0f * s.noEchoMeasurements / s.sequence : 0.0f;
  Serial.printf(
    "%s | %s | %s | HEALTH=%s DATA=%s RAW=%.1f MED=%.1f EMA=%.1f LAST=%.1f HELD=%.1f cm | "
    "FAIL=%u RECOVERING=%u/%u AGE=%lu ms NO_ECHO=%.1f%% FAULTS=%lu ALERT=%d CAUTION=%d FAULT_LATCH=%d REASON=%s\n",
    name, measurementName(i), clearanceName(i), status,
    sonarDataName(i), s.raw, s.median, s.filtered, s.lastValidRaw,
    !s.latestValid && s.validMeasurements && millis() - s.lastGood <= SONAR_HOLD_MS ? s.filtered : NAN,
    s.consecutiveNoEcho, min((int)s.consecutiveValid, (int)SONAR_RECOVERY_SAMPLES),
    SONAR_RECOVERY_SAMPLES,
    s.validMeasurements ? (unsigned long)(millis() - s.lastGood) : 0UL,
    noEchoRate, (unsigned long)s.faultMeasurements, s.nearLatched, s.cautionLatched, s.faultLatched, s.reason
  );
  if (mode == FOLLOW_LINE && i != FRONT && sonarClearance(i) == ClearanceState::UNKNOWN &&
      !s.nearLatched && !s.faultLatched)
    Serial.printf("%s ACTION=IGNORE_SIDE_UNKNOWN_WHILE_LINE\n", name);
}

void logSonarStatus(bool grouped = true) {
  if (grouped) beginLog("SONAR STATUS");
  printSonar(FRONT, "FRONT"); printSonar(LEFT, "LEFT"); printSonar(RIGHT, "RIGHT");
  if (grouped) endLog();
}

void handleCommand(String cmd) {
  cmd.trim();
  cmd.toUpperCase();

  if (cmd == "S" || cmd == "M") {
    setMode(MANUAL, "STOP_COMMAND");
    stopMotors();
    logMessage("STOP", "STOP / MANUAL");
  }

  else if (cmd == "A") {
    stopMotors();
    if (mode == ABORTED) {
      logMessage("LOCKED", "LOCKED: inspect robot, send S then A");
      return;
    }
    setMode(MANUAL, "AUTO_REINITIALIZE");

    Sensors s = readRaw();

    stable = s;
    rawPrev = s;

    for (int i = 0; i < 3; i++) {
      changedAt[i] = millis();
    }

    // FOLLOW_LINE handles initialization and transient faults without requiring another A.
    frontNearCount = 0;
    lastFrontProcessed = sonar[FRONT].validSequence;

    setMode(FOLLOW_LINE, "AUTO_COMMAND");
  }

  else if (cmd == "D") {
    static uint32_t lastDebugAt = 0;
    static bool debugPrinted = false;
    if (debugPrinted && millis() - lastDebugAt < 250) return;
    lastDebugAt = millis();
    debugPrinted = true;
    Sensors s = readRaw();

    beginLog("SNAPSHOT");
    Serial.printf(
      "LINE LCR=%d%d%d | MODE=%d\n",
      s.l, s.c, s.r, (int)mode
    );

    logSonarStatus(false);
    Serial.printf("MODE=%s ACTION=%s FRONT_CONFIRM=%u/%u UNKNOWN_CAP=%d FAR_CAP=%d\n",
                  modeName(mode), controlAction, frontNearCount, FRONT_CONFIRM,
                  UNKNOWN_PROBE_PWM, NO_ECHO_FAR_PWM);
    Serial.printf("SIDE_MIN=%.1f BODY_MARGIN=%.1f AVOID=%s WAIT=%d\n",
                  SIDE_MIN_GAP_CM, edgeBodyMarginCm(),
                  avoidRight ? "RIGHT" : "LEFT", waitingClearance);
    Serial.printf("OFFSET saw=%d seen=%u clear=%u edge=%.1f moved=%.1f | BYPASS saw=%d clear=%u\n",
                  sawOffsetObject, offsetSeenCount, offsetClearCount,
                  offsetEdgeDistance, phaseDistanceCm(), sawWall, clearWallCount);
    printEncoders();
    endLog();
  }

  else if (cmd == "E") {
    beginLog("ENCODER");
    printEncoders();
    endLog();
  }

  else if (cmd == "Z") {
    if (mode != MANUAL || currentLeft != 0 || currentRight != 0) {
      logMessage("COMMAND", "STOP FIRST: send S before resetting encoders");
      return;
    }
    portENTER_CRITICAL(&encoderMux);
    encoderLeftPulses = encoderRightPulses = 0;
    portEXIT_CRITICAL(&encoderMux);
    resetEncoderPhase();
    logMessage("ENCODER", "ENCODERS ZEROED");
  }

  else if (
    cmd == "F" ||
    cmd == "B" ||
    cmd == "L" ||
    cmd == "R"
  ) {
    setMode(MANUAL, "MANUAL_DRIVE_COMMAND");

    if (cmd == "F") {
      drive(baseSpeed, baseSpeed);
    }
    else if (cmd == "B") {
      drive(-baseSpeed, -baseSpeed);
    }
    else if (cmd == "L") {
      drive(-baseSpeed, baseSpeed);
    }
    else if (cmd == "R") {
      drive(baseSpeed, -baseSpeed);
    }
    controlAction = "MANUAL_DRIVE";
    logMessage("MANUAL DRIVE", "Motor command applied");
  }

  else if (isNumber(cmd) && cmd.toInt() <= 255) {
    baseSpeed = cmd.toInt();

    beginLog("COMMAND");
    Serial.printf(
      "BASE SPEED=%d\n",
      baseSpeed
    );
    endLog();
  }

  else {
    logMessage("COMMAND", "UNKNOWN COMMAND");
  }
}

void pollSerialCommands() {
  static char buffer[32];
  static uint8_t length = 0;
  static bool overflow = false;
  static uint32_t lastByteAt = 0;
  auto dispatch = [&]() {
    if (overflow) logMessage("COMMAND", "COMMAND_TOO_LONG");
    else if (length) { buffer[length] = '\0'; handleCommand(String(buffer)); }
    length = 0; overflow = false;
  };
  // Bounded parsing; never wait synchronously for a newline.
  for (int budget = 0; budget < 16 && Serial.available(); budget++) {
    char c = (char)Serial.read();
    lastByteAt = millis();
    if (c == '\n' || c == '\r') dispatch();
    else if (length < sizeof(buffer) - 1) buffer[length++] = c;
    else overflow = true;
  }
  if ((length || overflow) && millis() - lastByteAt >= 20) dispatch();
}

// =====================================================
// SETUP
// =====================================================

void setup() {
  Serial.setTxBufferSize(4096); // Configure before begin: avoid blocking on log bursts.
  Serial.begin(115200);
  Serial.setTimeout(20);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  pinMode(LINE_LEFT, INPUT);
  pinMode(LINE_CENTER, INPUT);
  pinMode(LINE_RIGHT, INPUT);

  for (int i = 0; i < 3; i++) {
    pinMode(TRIG[i], OUTPUT);
    digitalWrite(TRIG[i], LOW);

    pinMode(ECHO[i], INPUT);
  }

  ledcAttach(ENA, 1000, 8);
  ledcAttach(ENB, 1000, 8);

  stopMotors();

  pinMode(ENCODER_LEFT_PIN, INPUT_PULLUP);
  pinMode(ENCODER_RIGHT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_PIN), encoderLeftISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_PIN), encoderRightISR, RISING);
  resetEncoderPhase();

  stable = readRaw();
  rawPrev = stable;

  beginLog("READY");
  Serial.println("ESP32 ROBOT READY");
  Serial.println("3 LINE + 3 HC-SR04 + 2 D0 ENCODERS");
  Serial.printf("ENC GPIO L=%u R=%u | PPR L=%.1f R=%.1f | TRACK=%.1f cm\n",
                ENCODER_LEFT_PIN, ENCODER_RIGHT_PIN,
                ENCODER_LEFT_PPR, ENCODER_RIGHT_PPR, WHEEL_TRACK_CM);
  Serial.println("FILTER: MEDIAN + EMA");
  Serial.println("NO_ECHO=UNKNOWN: limited FOLLOW_LINE allowed; never proves avoidance clearance");
  Serial.printf("UNKNOWN startup=%d cap=%d | after-far=%d cap=%d; calibrate braking on chassis\n",
                ALLOW_UNKNOWN_STARTUP, UNKNOWN_PROBE_PWM, ALLOW_NO_ECHO_AFTER_FAR, NO_ECHO_FAR_PWM);
  Serial.println("A=AUTO S=STOP D=DEBUG E=ENCODERS Z=ZERO (stopped only)");
  endLog();
}

// =====================================================
// LOOP
// =====================================================

void loop() {
  pollSonar();
  updateLine();

  pollSerialCommands();

  if (millis() - lastControl >= CONTROL_MS) {
    lastControl = millis();
    control();
    logControlAction();
  }
}
