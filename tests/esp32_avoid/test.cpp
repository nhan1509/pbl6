#include "Arduino.h"
#include <cassert>
#include <iostream>
#include <sstream>
uint32_t fakeNow = 1000;
int pins[40] = {};
SerialStub Serial;
#include "../../esp32_line_3sensor_ultrasonic_avoid/esp32_line_3sensor_ultrasonic_avoid.ino"
float leftFraction=0, rightFraction=0;

void echo(int i, float cm) {
  addSonarSample(i, (unsigned long)(cm*2.0f/0.0343f+0.5f));
}
void measure(int i,float cm) { if(cm==0)addSonarSample(i,0);else echo(i,cm); }
void simulateWheels(uint32_t ms, bool leftWorking=true, bool rightWorking=true) {
  leftFraction += leftWorking ? abs(currentLeft)*0.00025f*ms/leftPulseCm() : 0;
  rightFraction += rightWorking ? abs(currentRight)*0.00025f*ms/rightPulseCm() : 0;
  uint32_t l=(uint32_t)leftFraction, r=(uint32_t)rightFraction;
  encoderLeftPulses+=l; encoderRightPulses+=r;
  leftFraction-=l; rightFraction-=r;
}
void step(uint32_t ms=10,float front=40,float left=25,float right=35,bool simulate=true) {
  if(simulate)simulateWheels(ms);
  fakeNow+=ms;
  measure(FRONT,front);measure(LEFT,left);measure(RIGHT,right);control();
}
void reset() {
  fakeNow=1000;encoderLeftPulses=encoderRightPulses=0;
  encoderLeftEdgeUs=encoderRightEdgeUs=0;leftFraction=rightFraction=0;
  for(auto &s:sonar)s=SonarFilter();
  for(int k=0;k<3;k++){echo(FRONT,40);echo(LEFT,25);echo(RIGHT,35);}
  stopMotors();frontNearCount=0;lastFrontProcessed=sonar[FRONT].validSequence;
  avoidSince=fakeNow;setMode(FOLLOW_LINE);resetSideChecks();
  stable=rawPrev={false,true,false};pins[LINE_CENTER]=HIGH;
  for(auto &t:changedAt)t=fakeNow;
  sawWall=false;seenWallCount=clearWallCount=lineCount=0;
  wallLastSeenAt=0;avoidRight=true;wallSide=LEFT;
}
void choose(float left,float right) {
  reset();step(80,9,left,right);assert(mode==FOLLOW_LINE && currentLeft==0);
  step(80,9,left,right);assert(mode==CHECK_SIDES);
  control();assert(mode==CHECK_SIDES);
  step(160,9,left,right);step(160,9,left,right);assert(mode==TURN_OUT);
}

// Sonar world: different obstacle widths must produce different offset lengths.
// Sensor readings, wheel pulses and 10ms controller ticks are independent.
float runDetour(bool leftDetour,float obstacleEdgeCm,bool bypassVisible=true) {
  reset(); measuredOffsetCm=0; bool visited[ABORTED+1]={};
  const uint8_t schedule[4]={FRONT,LEFT,FRONT,RIGHT}; unsigned slot=0;
  for(int tick=0;tick<5000;tick++) {
    simulateWheels(10); fakeNow+=10;
    bool approach=mode==FOLLOW_LINE || mode==CHECK_SIDES;
    bool lineVisible=approach || mode==ALIGN_HEADING || mode==ALIGN_LINE ||
                     (mode==FIND_LINE && phaseDistanceCm()>=measuredOffsetCm);
    pins[LINE_CENTER]=lineVisible?HIGH:LOW; updateLine();
    if(tick%6==0) {
      uint8_t i=schedule[slot++%4]; float cm=130;
      if(approach && i==FRONT)cm=9;
      if(approach && !leftDetour && i==LEFT)cm=25;
      if(approach && leftDetour && i==RIGHT)cm=25;
      if(mode==TURN_OUT && i==FRONT && phaseDistanceCm()<turnWheelCm()*0.45f)cm=9;
      if(mode==OFFSET_OUT && i==wallSide && phaseDistanceCm()<obstacleEdgeCm)cm=10;
      if(mode==BYPASS && bypassVisible && i==wallSide && phaseDistanceCm()<32)cm=20;
      addSonarSample(i,cm==0?0:(unsigned long)(cm*2/0.0343f));
    }
    control(); visited[mode]=true;
    if(mode==ABORTED) {
      assert(!bypassVisible && visited[BYPASS]); return measuredOffsetCm;
    }
    if(mode==FOLLOW_LINE && visited[ALIGN_LINE]) {
      assert(bypassVisible && avoidRight!=leftDetour);
      assert(visited[OFFSET_OUT] && visited[TURN_PARALLEL] && visited[BYPASS]);
      assert(visited[FORWARD_CLEAR] && visited[TURN_IN] && visited[FIND_LINE] && visited[ALIGN_HEADING]);
      assert(measuredOffsetCm>=obstacleEdgeCm+edgeBodyMarginCm()); return measuredOffsetCm;
    }
  }
  assert(false && "Sonar-edge detour did not complete");return 0;
}
void settle(float front=100,float left=100,float right=100) {
  for(int k=0;k<25;k++)step(10,front,left,right,false);
  if(settlingMotion)std::cerr<<"Settle failed mode="<<modeName(mode)<<" waiting="<<waitingClearance
    <<" front="<<sonarStatusName(FRONT)<<" left="<<sonarStatusName(LEFT)<<" right="<<sonarStatusName(RIGHT)<<"\n";
  assert(!settlingMotion);
}
void freshOtherSensors() { echo(LEFT,100); echo(RIGHT,100); }
void clearFront() { sonar[FRONT]=SonarFilter();frontNearCount=0;lastFrontProcessed=0; }
void assertGroupedLogs(const std::string &log) {
  std::istringstream input(log);std::string line;bool inside=false, title=false;
  unsigned groups=0;
  while(std::getline(input,line)) {
    if(line=="--------------------------------------------------") {
      if(inside){assert(title);groups++;}else title=false;
      inside=!inside;
    }else if(!line.empty()) {
      assert(inside && "Unframed Serial output");
      if(!title){assert(line[0]=='[');title=true;}
    }
  }
  assert(!inside && groups>0);
}
#ifndef ENCODER_MONITOR_ONLY
int main() {
  // 1. Valid history/EMA are held on loss, never replaced with 200 or a sample.
  reset();clearFront();echo(FRONT,15);echo(FRONT,16);
  float ema=sonar[FRONT].filtered, med=sonar[FRONT].median;
  uint8_t count=sonar[FRONT].count; uint32_t validSeq=sonar[FRONT].validSequence;
  addSonarSample(FRONT,0);
  assert(sonar[FRONT].status==SONAR_TEMP_LOSS && !isfinite(sonar[FRONT].raw));
  assert(sonar[FRONT].filtered==ema && sonar[FRONT].median==med && sonar[FRONT].count==count);
  assert(sonar[FRONT].validSequence==validSeq && !confirmedClear(FRONT,10));
  echo(FRONT,15);assert(!confirmedClear(FRONT,10));
  echo(FRONT,14);assert(sonarOk(FRONT) && sonar[FRONT].consecutiveNoEcho==0);

  // A previous caution-range echo keeps the car stopped across timeouts,
  // without falsely classifying ordinary NO_ECHO as FAULT/LOST.
  reset();clearFront();echo(FRONT,15);control();
  for(int k=0;k<10;k++) {
    fakeNow+=240;freshOtherSensors();addSonarSample(FRONT,0);control();
    assert(currentLeft==0 && currentRight==0);
    if(k<3)assert(mode==FOLLOW_LINE);
  }
  assert(sonar[FRONT].status==SONAR_TEMP_LOSS && mode==FOLLOW_LINE);
  assert(sonar[FRONT].measurement==EchoMeasurement::NO_ECHO && sonar[FRONT].cautionLatched);

  // 3. A new 8cm echo stops immediately although MED/EMA still say ~50cm.
  reset();clearFront();echo(FRONT,50);echo(FRONT,49);control();
  echo(FRONT,8);control();
  assert(sonar[FRONT].filtered>30 && currentLeft==0 && frontNearCount==1);

  // 4. NO_ECHO must not erase a pending near alert; recovery is still mandatory.
  reset();clearFront();echo(FRONT,8);control();assert(frontNearCount==1);
  addSonarSample(FRONT,0);control();addSonarSample(FRONT,0);control();
  assert(frontNearCount==1 && !confirmedClear(FRONT,10));
  echo(FRONT,9);control();assert(currentLeft==0 && !sonarOk(FRONT) && mode==FOLLOW_LINE);
  echo(FRONT,9);control();assert(mode==CHECK_SIDES);

  // 5. Initial losses -> returning 40,42,41: first echo cannot prove clearance.
  reset();clearFront();addSonarSample(FRONT,0);addSonarSample(FRONT,0);
  echo(FRONT,40);assert(!confirmedClear(FRONT,30));
  echo(FRONT,42);assert(confirmedClear(FRONT,30));
  echo(FRONT,41);assert(confirmedClear(FRONT,30));

  // 6. LEFT loss in BYPASS cannot produce an edge, even after seeing the wall.
  reset();setMode(BYPASS);settle(100,20,100);sawWall=true;clearWallCount=2;
  step(240,100,0,100);assert(mode==BYPASS && waitingClearance && clearWallCount==0);
  assert(currentLeft==0 && currentRight==0);

  // 7. RIGHT loss in CHECK_SIDES cannot authorize a right turn.
  reset();setMode(CHECK_SIDES);resetSideChecks();
  step(240,9,100,0);step(240,9,100,0);
  assert(mode==CHECK_SIDES && sideReadyCount[1]==0 && currentLeft==0);

  // 8. Stable echoes support line following and full variable-width detours.
  reset();for(int k=0;k<20;k++){step(120,100,100,100);assert(mode==FOLLOW_LINE && !waitingClearance);}
  float narrow=runDetour(false,12), wide=runDetour(false,28);
  assert(wide>narrow+10);runDetour(true,12);runDetour(true,28);

  // Every moving avoidance mode stops on missing required side clearance and
  // stays in that mode during a temporary loss; one return echo cannot restart.
  const Mode modes[]={TURN_OUT,OFFSET_OUT,TURN_PARALLEL,BYPASS,FORWARD_CLEAR,
                      TURN_IN,FIND_LINE,ALIGN_HEADING,ALIGN_LINE};
  for(Mode m:modes) {
    reset();stable={false,false,false};setMode(m);settle(100,100,100);
    step(60,100,0,100,false);
    assert(mode==m && waitingClearance && currentLeft==0 && currentRight==0);
    step(60,100,100,100,false);
    assert(mode==m && waitingClearance && currentLeft==0 && currentRight==0);
  }

  // Held edge data cannot survive loss/recovery as an already-confirmed edge.
  reset();setMode(OFFSET_OUT);settle(100,10,100);sawOffsetObject=true;
  offsetClearCount=3;offsetEdgeDistance=0;
  step(240,100,0,100);assert(offsetClearCount==0 && !isfinite(offsetEdgeDistance));

  // Long loss re-seeds MED/EMA from the new real level, but still needs recovery.
  reset();clearFront();echo(FRONT,8);echo(FRONT,8);
  for(int k=0;k<3;k++){fakeNow+=240;addSonarSample(FRONT,0);}
  echo(FRONT,100);assert(fabsf(sonar[FRONT].filtered-100)<0.1f && !sonarOk(FRONT));
  echo(FRONT,102);assert(sonarOk(FRONT));

  // Median rejects one valid distant outlier; raw still handles close hazards.
  reset();clearFront();echo(FRONT,15);echo(FRONT,15);echo(FRONT,15);echo(FRONT,150);
  assert(sonar[FRONT].median<16 && sonar[FRONT].filtered<16);
  addSonarSample(FRONT,12001);assert(sonar[FRONT].status==SONAR_FAULT && !confirmedClear(FRONT,10));
  echo(FRONT,4);control();assert(mode==ABORTED); // Emergency during recovery.
  reset();float oldMedian=sonar[FRONT].median;uint32_t oldValid=sonar[FRONT].validSequence;
  addSonarSample(FRONT,20);control(); // ~0.34cm abnormal pulse, not a valid sample.
  assert(sonar[FRONT].status==SONAR_FAULT && sonar[FRONT].median==oldMedian);
  assert(sonar[FRONT].validSequence==oldValid && currentLeft==0 && currentRight==0);

  // Threshold-specific clearance requires BOTH recent raw samples above it.
  reset();clearFront();echo(FRONT,8);echo(FRONT,100);
  assert(!confirmedClear(FRONT,20));
  echo(FRONT,100);echo(FRONT,100);assert(confirmedClear(FRONT,20));

  // Schedule-compatible freshness: a side is fresh just before its next 240ms slot.
  reset();fakeNow+=239;assert(sonarOk(LEFT));
  fakeNow+=SENSOR_STALE_MS;assert(!sonarOk(LEFT) && sonar[LEFT].status==SONAR_LOST);
  reset();fakeNow=UINT32_MAX-100;sonar[LEFT]=SonarFilter();echo(LEFT,40);echo(LEFT,40);
  fakeNow=100;assert(sonarOk(LEFT)); // millis rollover.

  // pollSonar: 60ms spacing, FRONT/LEFT/FRONT/RIGHT, and stuck-high detection.
  reset();for(auto &s:sonar)s=SonarFilter();lastSonar=fakeNow;nextSonar=0;
  fakeNow+=59;pollSonar();assert(sonar[FRONT].sequence==0);
  fakeNow++;pollSonar();assert(sonar[FRONT].sequence==1);
  fakeNow+=60;pollSonar();assert(sonar[LEFT].sequence==1);
  fakeNow+=60;pollSonar();assert(sonar[FRONT].sequence==2);
  pins[ECHO[RIGHT]]=HIGH;fakeNow+=60;pollSonar();
  assert(sonar[RIGHT].measurement==EchoMeasurement::NO_ECHO && !sonar[RIGHT].faultLatched);
  fakeNow+=ECHO_HIGH_FAULT_MS;pollSonar();
  assert(sonar[RIGHT].status==SONAR_FAULT);pins[ECHO[RIGHT]]=LOW;
  reset();nextSonar=0;lastSonar=fakeNow;pulseHighAfterTimeout()=ECHO[FRONT];
  fakeNow+=60;pollSonar();pulseHighAfterTimeout()=-1;
  assert(sonar[FRONT].measurement==EchoMeasurement::NO_ECHO && sonar[FRONT].pendingHigh);
  fakeNow+=20;pins[ECHO[FRONT]]=LOW;pollSonar();
  assert(!sonar[FRONT].faultLatched && !sonar[FRONT].pendingHigh); // Far pulse ended normally.
  reset();nextSonar=0;lastSonar=fakeNow;pulseHighAfterTimeout()=ECHO[FRONT];
  fakeNow+=60;pollSonar();pulseHighAfterTimeout()=-1;
  fakeNow+=ECHO_HIGH_FAULT_MS;pollSonar();
  assert(sonar[FRONT].measurement==EchoMeasurement::SIGNAL_FAULT && sonar[FRONT].faultLatched);
  pins[ECHO[FRONT]]=LOW;

  // Existing encoder guards remain: no pulses cannot finish an offset.
  reset();setMode(OFFSET_OUT);
  for(int k=0;k<350 && mode!=ABORTED;k++)step(10,100,10,100,false);
  assert(mode==ABORTED);
  reset();setMode(TURN_PARALLEL);settle();encoderLeftPulses+=5;
  step(10,100,100,100,false);assert(mode==ABORTED);
  // New request #1: far echo -> continuous NO_ECHO keeps limited line following.
  reset();clearFront();echo(FRONT,150);control();
  for(int k=0;k<25;k++) {
    step(240,0,100,100);
    assert(mode==FOLLOW_LINE && !waitingClearance && currentLeft>0 && currentRight>0);
    assert(currentLeft<=NO_ECHO_FAR_PWM && currentRight<=NO_ECHO_FAR_PWM);
    assert(sonarClearance(FRONT)==ClearanceState::UNKNOWN && !confirmedClear(FRONT,10));
    assert(sonar[FRONT].status!=SONAR_LOST && !sonar[FRONT].faultLatched);
  }

  // #2: exact supplied log, LEFT times out 100%, FRONT=90 and RIGHT=28.
  reset();sonar[LEFT]=SonarFilter();
  for(int k=0;k<25;k++){step(240,90,0,28);assert(mode==FOLLOW_LINE && !waitingClearance && currentLeft>0);}

  // #3: all sensors NO_ECHO from startup; probe is configurable and never CLEAR.
  reset();for(auto &s:sonar)s=SonarFilter();lastFrontProcessed=0;
  for(int k=0;k<25;k++) {
    step(240,0,0,0);
    assert(mode==FOLLOW_LINE && currentLeft>0 && currentRight>0);
    assert(currentLeft<=UNKNOWN_PROBE_PWM && currentRight<=UNKNOWN_PROBE_PWM);
    assert(sonarClearance(FRONT)==ClearanceState::UNKNOWN);
  }
  // Steering boosts, PWM ramp-down and encoder trim cannot defeat the cap.
  stable={true,false,false};drive(200,200);step(10,0,0,0,false);
  assert(currentLeft<=UNKNOWN_PROBE_PWM && currentRight<=UNKNOWN_PROBE_PWM);
  const Sensors patterns[]={{false,true,false},{true,true,false},{true,false,false},
                             {false,true,true},{false,false,true},{true,true,true}};
  for(auto pattern:patterns) {
    stable=pattern;
    for(int k=0;k<20;k++)step(10,0,0,0);
    assert(abs(currentLeft)<=UNKNOWN_PROBE_PWM && abs(currentRight)<=UNKNOWN_PROBE_PWM);
  }

  // #4: 9cm -> NO_ECHO remains stopped indefinitely until new real clearance.
  reset();clearFront();echo(FRONT,9);control();
  for(int k=0;k<20;k++) {step(240,0,100,100);assert(mode==FOLLOW_LINE && currentLeft==0 && currentRight==0);}
  assert(sonar[FRONT].nearLatched && frontNearCount==1);
  echo(FRONT,100);control();assert(currentLeft==0);
  echo(FRONT,100);control();assert(!sonar[FRONT].nearLatched && currentLeft>0);

  // #5: a new near echo interrupts probing on the very first valid sample.
  reset();clearFront();addSonarSample(FRONT,0);control();echo(FRONT,8);control();
  assert(currentLeft==0 && currentRight==0 && frontNearCount==1);

  // #6/#7: the earlier BYPASS and CHECK_SIDES tests remain strict.
  // #8: exact 15,80,16 sequence, median must reject the isolated 80cm spike.
  reset();clearFront();echo(FRONT,15);echo(FRONT,80);echo(FRONT,16);
  assert(fabsf(sonar[FRONT].median-16)<0.1f);
  assert(sonarClearance(FRONT)==ClearanceState::NEAR);

  // #9: stable valid echoes and all four full detours were exercised above.
  // #10: known signal faults cannot be hidden by subsequent NO_ECHO.
  reset();addSonarSample(LEFT,0,true);step(60,90,0,28);
  assert(sonar[LEFT].faultLatched && mode==FOLLOW_LINE && currentLeft>0);
  assert(currentLeft<=DEGRADED_FOLLOW_PWM && currentRight<=DEGRADED_FOLLOW_PWM);
  step(60,90,100,28);assert(sonar[LEFT].faultLatched && currentLeft>0);
  step(60,90,100,28);assert(!sonar[LEFT].faultLatched);
  for(int k=0;k<3;k++)step(60,90,100,28);
  assert(mode==FOLLOW_LINE && currentLeft>0);

  // An old side collision alert also survives timeout and S/A until fresh clearance.
  reset();step(10,100,3,100);assert(mode==ABORTED);
  addSonarSample(LEFT,0);handleCommand(String("S"));handleCommand(String("A"));control();
  assert(mode==FOLLOW_LINE && currentLeft==0 && sonar[LEFT].nearLatched);
  // Comprehensive request: >190cm echoes within timeout are far valid echoes,
  // not hardware faults. Actual inconsistent pulses still carry a reason.
  reset();addSonarSample(RIGHT,11500);assert(sonar[RIGHT].measurement==EchoMeasurement::ECHO_VALID);
  assert(!sonar[RIGHT].faultLatched && sonar[RIGHT].raw>190);
  addSonarSample(RIGHT,12001);assert(strcmp(sonar[RIGHT].reason,"PULSE_EXCEEDS_TIMEOUT")==0);

  // A transient RIGHT fault cannot stop/abort the entire line controller.
  reset();addSonarSample(RIGHT,0,true,"ECHO_HIGH_BEFORE_TRIGGER");
  for(int k=0;k<30;k++)step(120,90,100,0);
  assert(mode==FOLLOW_LINE && currentLeft>0 && currentRight>0);
  step(120,90,100,28);step(120,90,100,28);
  assert(!sonar[RIGHT].faultLatched && mode==FOLLOW_LINE);

  // Front fault: retry temporarily, auto-recover, and hold for persistent faults.
  reset();addSonarSample(FRONT,0,true);control();assert(currentLeft==0 && mode==FOLLOW_LINE);
  step(120,90,100,28);step(120,90,100,28);assert(currentLeft>0 && mode==FOLLOW_LINE);
  reset();for(int k=0;k<7;k++) {
    fakeNow+=120;freshOtherSensors();addSonarSample(FRONT,0,true);control();
    assert(mode==FOLLOW_LINE && currentLeft==0);
  }

  // Symmetric slew avoids the former backwards=45 / forwards=12 spin imbalance.
  assert(rampPWM(0,85)==12 && rampPWM(0,-85)==-12);
  assert(rampPWM(-85,0)==-40 && rampPWM(85,0)==40);
  assert(rampPWM(-85,85)==-40); // Reverse via zero, never swap sign instantly.

  // All LCR patterns, including explicit ambiguous/intersection policies.
  const Sensors linePatterns[]={{false,true,false},{true,false,false},{false,false,true},
                                {true,true,false},{false,true,true},{true,true,true},{true,false,true}};
  for(unsigned p=0;p<7;p++) {
    reset();stable=linePatterns[p];
    for(int k=0;k<15;k++)step(10,100,100,100);
    if(p==1 || p==3)assert(currentLeft<currentRight);
    else if(p==2 || p==4)assert(currentLeft>currentRight);
    else assert(currentLeft==currentRight && currentLeft>0);
  }
  reset();for(int k=0;k<10;k++)step(10,100,100,100);
  stable={false,false,false};
  for(int k=0;k<4;k++)step(10,100,100,100);
  assert(currentLeft>0 && currentRight>0 && recoveringLine);
  stable={false,true,false};for(int k=0;k<3;k++)step(10,100,100,100);
  assert(!recoveringLine && mode==FOLLOW_LINE);
  stable={true,false,false};for(int k=0;k<10;k++)step(10,100,100,100);
  stable={false,false,false};
  for(int k=0;k<40;k++)step(10,100,100,100);
  assert(currentLeft==0 && currentRight==0 && mode==FOLLOW_LINE);
  stable={false,true,false};for(int k=0;k<3;k++)step(10,100,100,100);
  assert(currentLeft>0 && mode==FOLLOW_LINE);

  // FIND_LINE rejects a one-tick hit before restoring heading.
  reset();stable={false,false,false};setMode(FIND_LINE);settle();
  stable={false,true,false};step(10,100,100,100,false);assert(mode==FIND_LINE);
  stable={false,false,false};step(10,100,100,100,false);assert(lineCount==0);
  stable={false,true,false};for(int k=0;k<3;k++)step(10,100,100,100,false);
  assert(mode==ALIGN_HEADING);

  // A chosen side needs turning room; the opposite needs only sweep clearance.
  choose(6,100);assert(avoidRight && mode==TURN_OUT);

  // Opposite unknown cannot deadlock translation recovery. Wall-side loss can.
  reset();setMode(BYPASS);settle(100,20,100);
  step(60,100,20,0);assert(!waitingClearance && mode==BYPASS);
  step(60,100,0,0);assert(waitingClearance && currentLeft==0);
  for(int k=0;k<3;k++)step(60,100,20,0);
  assert(!waitingClearance && mode==BYPASS && currentLeft>0);

  // D and every command/event group must begin and end with the exact separator.
  reset();fakeNow+=300;Serial.output.clear();handleCommand(String("D"));
  assert(Serial.output.find("[SNAPSHOT]")!=std::string::npos);
  assert(Serial.output.find("REASON=")!=std::string::npos);
  assertGroupedLogs(Serial.output);
  Serial.output.clear();
  handleCommand(String("S"));handleCommand(String("E"));handleCommand(String("Z"));
  handleCommand(String("F"));handleCommand(String("B"));handleCommand(String("L"));handleCommand(String("R"));
  handleCommand(String("S"));handleCommand(String("70"));handleCommand(String("invalid"));
  handleCommand(String("A"));logControlAction();abortAvoid("TEST_ABORT");
  handleCommand(String("A"));setup();assertGroupedLogs(Serial.output);
  Serial.output.clear();Serial.input="S\n70\n";Serial.readAt=0;pollSerialCommands();
  assert(mode==MANUAL && baseSpeed==70);assertGroupedLogs(Serial.output);
  Serial.output.clear();Serial.input="F";Serial.readAt=0;pollSerialCommands();
  assert(currentLeft==0);fakeNow+=20;pollSerialCommands();assert(currentLeft==baseSpeed);
  assertGroupedLogs(Serial.output);
  Serial.output.clear();Serial.input=std::string(40,'X')+"\n";Serial.readAt=0;
  for(int k=0;k<4;k++)pollSerialCommands();
  assert(Serial.output.find("COMMAND_TOO_LONG")!=std::string::npos);assertGroupedLogs(Serial.output);
  std::cout<<"PASS: all 15 comprehensive scenarios, prior sonar regressions, 4 complete detours, 9 avoidance modes, all LCR patterns, recovery/deadlock checks, far/invalid pulses, symmetric slew, grouped Serial snapshots/commands\n";
}
#endif
