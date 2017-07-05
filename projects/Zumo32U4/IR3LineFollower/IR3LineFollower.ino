/**
 * Created by Olavi Kamppari on 7/4/2017.
 */

/**
 * Demonstrate the classic linefollower using a single sensor (front) and
 * integer PID control.  The two other Zumo line sensors are used to do 
 * sharp turns to allow the PID control to run on lower kP value.
 * 
 * The selection of tracking the left or right edge of the tape is done 
 * during robot start.
 * 
 * The robot will end after 20,000 ms or when the left and right line sensor
 * see the tape at the same time.
 */

#include <Zumo32U4.h>
#include <ZumoIR3.h>

#define MAXSPEED 400
#define DEFSPEED 300
#define SP 50
#define kP 1200
#define kI 30
#define kD 300
#define BRAKEFACTOR 65
#define ACCELFACTOR 35
#define LOOPTIME 5
#define RUNTIME 20000
#define CVLOW   -550
#define CVHIGH  550

Zumo32U4Buzzer  buzzer;
Zumo32U4Motors  motors;
Zumo32U4ButtonA buttonA;
Zumo32U4ButtonB buttonB;
Zumo32U4LCD     lcd;
ZumoIR3         IR3;

int16_t error, prevError, dError;   // Control Values
int32_t pTerm, iTerm, dTerm;
int32_t prevTime, nowTime, dt;
int32_t cv;
int16_t slowSpeed, highSpeed;
int32_t exitTime;                   // Time when line was lost
int32_t startTime;

int16_t left, front, right;         // Sensor values

enum LFState {
  followRight,
  followLeft,
  waiting
};
LFState   lfState;

void initPID() {
  iTerm = 0;
  prevError = 0;
  prevTime = millis();
  exitTime = prevTime;
}

void setup() {
  Serial.begin(230400);
  // Play a little welcome song
  buzzer.play(">g32>>c32");

  // Wait for button A or B to be pressed and released.
  lcd.clear();
  lcd.print(F("A: Left"));
  lcd.gotoXY(0, 1);
  lcd.print(F("B: Right"));
  lfState = waiting;
  do {
    if (buttonA.isPressed()) lfState = followLeft;
    if (buttonB.isPressed()) lfState = followRight;
  } while (lfState == waiting);
  lcd.clear();
  lcd.print(F("Go!"));
  buzzer.play("L16 cdegreg4");
  while (buzzer.isPlaying());
  initPID();
  Serial.println(F("dt\tfront\terr\tdErr\tpT\tdT\tiT\tcv\tsSp\thSp\ttLeft"));
  startTime = millis();
}

void executePID() {
  bool turnLeft = (lfState == followRight);
  nowTime = millis();
  dt = nowTime - prevTime;
  prevTime = nowTime;
  if (2 * dt  < LOOPTIME) {
    prevError   = SP - front;
  } else {
    error       = SP - front;
    dError      = error - prevError;
    prevError   = error;
    pTerm       = (kP * error) / 100;
    dTerm       = (kD * dError) / dt;
    iTerm       += (kI * error * dt) / 1000;
    if (iTerm < -DEFSPEED) iTerm = -DEFSPEED;
    if (iTerm > (MAXSPEED - DEFSPEED)) iTerm = (MAXSPEED - DEFSPEED);
    cv          = pTerm + dTerm + iTerm;
    if (cv > 0) {
      if (cv > CVHIGH) cv = CVHIGH;
      slowSpeed   = DEFSPEED - BRAKEFACTOR * cv / 100;
      highSpeed   = DEFSPEED + ACCELFACTOR * cv / 100;
    } else {
      if (cv < CVLOW) cv = CVLOW;
      turnLeft = !turnLeft;
      slowSpeed   = DEFSPEED + BRAKEFACTOR * cv / 100;
      highSpeed   = DEFSPEED - ACCELFACTOR * cv / 100;
    }
    if (turnLeft) {
      motors.setSpeeds(slowSpeed, highSpeed);
    } else {
      motors.setSpeeds(highSpeed, slowSpeed);
    }
  }
  //    Serial.print(dt);           Serial.print('\t');
  //    Serial.print(front);        Serial.print('\t');
  //    Serial.print(error);        Serial.print('\t');
  //    Serial.print(dError);       Serial.print('\t');
  //    Serial.print(pTerm);        Serial.print('\t');
  //    Serial.print(dTerm);        Serial.print('\t');
  //    Serial.print(iTerm);        Serial.print('\t');
  //    Serial.print(cv);           Serial.print('\t');
  //    Serial.print(slowSpeed);    Serial.print('\t');
  //    Serial.print(highSpeed);    Serial.print('\t');
  //    Serial.print(turnLeft);        Serial.print('\n');

}

void loop() {
  left  = IR3.readLeft();
  front = IR3.readFront();
  right = IR3.readRight();
  //  Serial.print(left);   Serial.print('\t');
  //  Serial.print(front);  Serial.print('\t');
  //  Serial.print(right);  Serial.print('\n');
  if (left > 50 || right > 50) {
    if (left > 75 && right > 75) {
      motors.setSpeeds(0, 0);
      while (1);                // Stop when lifted from floor
    }
    if (left > 75)  motors.setSpeeds(-DEFSPEED,  DEFSPEED);
    if (right > 75) motors.setSpeeds( DEFSPEED, -DEFSPEED);
  } else {
    executePID();
  }
  delay(LOOPTIME);
  if (millis() - startTime > RUNTIME) {
    motors.setSpeeds(0, 0);
    while (1);
  }
}
