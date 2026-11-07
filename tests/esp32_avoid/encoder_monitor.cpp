#define ENCODER_MONITOR_ONLY
#include "test.cpp"

void monitorTick(int left, int right, bool leftPulse, bool rightPulse, uint32_t ms=10) {
  fakeNow+=ms;
  drive(left,right);
  if(leftPulse)encoderLeftPulses++;
  if(rightPulse)encoderRightPulses++;
  checkEncoderStall(fakeNow);
}

int main() {
  // 1. Straight driving: both pulse streams stay alive.
  reset();
  for(int k=0;k<400;k++) {
    monitorTick(70,70,k%10==0,k%10==0);
    assert(mode==FOLLOW_LINE);
  }
  assert(leftMonitor.status==WheelMotionStatus::WHEEL_MOVING);

  // 2/3. Deliberately stopped inside wheels have no pulse requirement.
  reset();stable={true,false,false};
  for(int k=0;k<400;k++)monitorTick(0,85,false,k%10==0);
  assert(mode==FOLLOW_LINE && leftMonitor.status==WheelMotionStatus::WHEEL_IDLE);
  reset();stable={false,false,true};
  for(int k=0;k<400;k++)monitorTick(85,0,k%10==0,false);
  assert(mode==FOLLOW_LINE && rightMonitor.status==WheelMotionStatus::WHEEL_IDLE);

  // 4/5. PWM 45 is ABOVE the old 40 threshold but sparse 2-second inside
  // pulses are legitimate. The active outer wheel remains supervised.
  reset();stable={true,true,false};
  for(int k=0;k<600;k++)monitorTick(45,85,k%200==0,k%10==0);
  assert(mode==FOLLOW_LINE && leftMonitor.status==WheelMotionStatus::WHEEL_LOW_SPEED);
  assert(rightMonitor.significant);
  reset();stable={false,true,true};
  for(int k=0;k<600;k++)monitorTick(85,45,k%10==0,k%200==0);
  assert(mode==FOLLOW_LINE && rightMonitor.status==WheelMotionStatus::WHEEL_LOW_SPEED);

  // Integration with the actual PD line controller: sharp/gentle left/right
  // keep the slow inside wheel pulse-free without inventing a zero-PWM command.
  for(bool right:{false,true}) {
    reset();stable=right?Sensors{false,true,true}:Sensors{true,true,false};
    for(int k=0;k<400;k++) {
      if(k%10==0){if(right)encoderLeftPulses++;else encoderRightPulses++;}
      step(10,100,100,100,false);assert(mode==FOLLOW_LINE);
    }
    assert((right?rightMonitor:leftMonitor).status==WheelMotionStatus::WHEEL_LOW_SPEED);
  }

  // 6. Ramp-up has a grace period; target is not confused with applied PWM.
  reset();
  for(int k=0;k<25;k++) {
    fakeNow+=10;smoothDrive(70,70);checkEncoderStall(fakeNow);
    assert(mode==FOLLOW_LINE);
    assert(leftMonitor.status!=WheelMotionStatus::WHEEL_STALL_CONFIRMED);
    if(k==0)assert(leftMonitor.status==WheelMotionStatus::WHEEL_ACCELERATING);
  }
  assert(targetLeftPWM==70 && currentLeft==70);

  // 7. Deceleration/intentional stop beats the stale previous PWM, even when
  // applied PWM is still 45 (>40) during the ramp-down step.
  reset();drive(90,70);checkEncoderStall(fakeNow);fakeNow+=2000;
  encoderRightPulses++;smoothDrive(0,70);checkEncoderStall(fakeNow);
  assert(mode==FOLLOW_LINE && targetLeftPWM==0 && currentLeft==45);
  assert(leftMonitor.status==WheelMotionStatus::WHEEL_IDLE);
  for(int k=0;k<300;k++)monitorTick(30,70,false,k%10==0);
  assert(mode==FOLLOW_LINE && leftMonitor.status==WheelMotionStatus::WHEEL_LOW_SPEED);

  // 8. Returning to straight resets the wheel's observation/rate window,
  // without resetting absolute counts or carrying old inside-curve intervals.
  reset();stable={true,true,false};
  for(int k=0;k<300;k++)monitorTick(45,85,false,k%10==0);
  uint32_t oldRight=encoderRightPulses;
  stable={false,true,false};monitorTick(70,70,false,true);
  assert(leftMonitor.requestSince==fakeNow && leftMonitor.pulseIntervalMs==0);
  assert(encoderRightPulses==oldRight+1);
  for(int k=0;k<25;k++)monitorTick(70,70,false,k%10==0);
  assert(mode==FOLLOW_LINE);
  for(int k=0;k<300 && mode!=ABORTED;k++)monitorTick(70,70,false,k%10==0);
  assert(mode==ABORTED && leftMonitor.status==WheelMotionStatus::WHEEL_STALL_CONFIRMED);

  // 9. Precise turn: a genuinely motionless driven wheel is confirmed stalled.
  reset();setMode(TURN_OUT);Serial.output.clear();
  for(int k=0;k<300 && mode!=ABORTED;k++)monitorTick(85,-85,false,k%10==0);
  assert(mode==ABORTED && currentLeft==0 && currentRight==0);
  assert(Serial.output.find("[ENCODER STALL]")!=std::string::npos);
  assert(Serial.output.find("LEFT_WHEEL_STALL")!=std::string::npos);
  assertGroupedLogs(Serial.output);

  // 10. A completed distance wheel stays intentionally stopped while the
  // other travels for >1500ms; phase counters/turn checks are not bypassed.
  reset();setMode(FORWARD_CLEAR);settle();
  encoderLeftPulses=phaseLeftStart+(uint32_t)ceilf(edgeBodyMarginCm()/leftPulseCm());
  int ticks=0;
  while(mode==FORWARD_CLEAR && ticks++<600) {
    simulateWheels(10,false,true);step(10,100,100,100,false);
    if(mode==FORWARD_CLEAR) {
      assert(targetLeftPWM==0 && currentLeft==0);
      assert(leftMonitor.status==WheelMotionStatus::WHEEL_IDLE);
    }
  }
  assert(mode==TURN_IN && ticks>150);

  // 11. No motion of BOTH wheels cannot hide behind changing curve contexts.
  reset();Serial.output.clear();
  for(int k=0;k<400 && mode!=ABORTED;k++) {
    stable=k%2?Sensors{true,true,false}:Sensors{false,true,true};
    monitorTick(50,50,false,false);
  }
  assert(mode==ABORTED && Serial.output.find("VEHICLE_NO_MOTION")!=std::string::npos);
  assertGroupedLogs(Serial.output);

  // 12. The first WAIT_CLEARANCE tick must stop before the watchdog evaluates
  // a >1500ms absence accumulated under a previous movement command.
  reset();setMode(BYPASS);settle(100,20,100);
  for(int k=0;k<80;k++)step(10,100,20,100,false);
  fakeNow+=2000;step(10,100,0,100,false);
  assert(mode==BYPASS && waitingClearance && targetLeftPWM==0 && targetRightPWM==0);
  assert(leftMonitor.status==WheelMotionStatus::WHEEL_IDLE);
  assert(rightMonitor.status==WheelMotionStatus::WHEEL_IDLE);

  // 13/14. Manual stop and latched abort do not produce subsequent stall logs.
  reset();handleCommand(String("S"));fakeNow+=10000;Serial.output.clear();
  control();checkEncoderStall(fakeNow);assert(mode==MANUAL && Serial.output.empty());
  reset();abortAvoid("TEST_ABORT");Serial.output.clear();fakeNow+=10000;
  control();checkEncoderStall(fakeNow);assert(mode==ABORTED && Serial.output.empty());

  // 15. D reports the complete monitor in an explicitly framed snapshot.
  reset();stable={false,false,true};monitorTick(85,0,true,false);
  fakeNow+=300;Serial.output.clear();handleCommand(String("D"));
  for(const char *field:{"MOTION:","TARGET_PWM:","ACTUAL_PWM:","PULSE_DELTA:","STATUS:","INTENTIONALLY_STOPPED"})
    assert(Serial.output.find(field)!=std::string::npos);
  assertGroupedLogs(Serial.output);

  // Adaptive timeout is tied to commanded/applied PWM and pulse distance,
  // not one enlarged fixed deadline. Coarser resolution gets a bounded margin.
  reset();monitorTick(70,70,false,false);uint32_t normal=leftMonitor.timeoutMs;
  reset();monitorTick(50,50,false,false);assert(leftMonitor.timeoutMs>normal);
  reset();updateWheelMonitor(leftMonitor,0,70,70,MotionContext::LINE_STRAIGHT,false,leftPulseCm()*2,fakeNow);
  assert(leftMonitor.timeoutMs>normal && leftMonitor.timeoutMs<=ENCODER_FOLLOW_MAX_MS);

  // A pulse during suspicion clears that suspicion before confirmation.
  reset();monitorTick(70,70,false,false);
  for(int k=0;k<160;k++)monitorTick(70,70,false,k%10==0);
  assert(leftMonitor.status==WheelMotionStatus::WHEEL_STALL_SUSPECTED);
  monitorTick(70,70,true,true);assert(leftMonitor.status==WheelMotionStatus::WHEEL_MOVING);
  assert(mode==FOLLOW_LINE);

  std::cout<<"PASS: all 15 encoder cases, actual PD corner integration, target/actual separation, adaptive resolution/timeouts, wait-before-watchdog, whole-car no-motion, stall confirmation/recovery, framed monitoring\n";
}
