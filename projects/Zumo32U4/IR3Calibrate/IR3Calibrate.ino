/**
 * Created by Olavi Kamppari on 7/4/2017.
 */

/**
 * Used to calibrate Zumo32U4 line sensor values.  This program uses line monitor
 * to print program lines for ZumoIR3.cpp.  By default, these lines are commented
 * out.  The active calibration values are selected by uncommenting them
 */

#include <Zumo32U4.h>
#include <ZumoIR3.h>

Zumo32U4Buzzer      buzzer;
Zumo32U4Motors      motors;
Zumo32U4Encoders    encoders;
Zumo32U4ButtonA     buttonA;
Zumo32U4ButtonB     buttonB;
Zumo32U4ButtonC     buttonC;
Zumo32U4LCD         lcd;
ZumoIR3             IR3;

#define HIST_SIZE   80

int16_t lineAve[3], bgndAve[3], edgeAve[3];
uint8_t lineHist[3][HIST_SIZE], bgndHist[3][HIST_SIZE], edgeHist[3][HIST_SIZE];
char    lineName[32], bgndName[32];

void getNames() {
  lcd.clear();
  lcd.print(F("Use Ser."));
  lcd.gotoXY(0, 1);
  lcd.print(F("Monitor"));
  while (Serial.available()) Serial.read();  // Flush before reading
  Serial.print(F("Enter the name of line and background: "));
  char x;
  while (!Serial.available());
  for (int i = 0; i < 31; i++) {
    x = Serial.read();
    if (x < ' ' || x == ',') {
      lineName[i] = 0;
      break;
    }
    lineName[i] = x;
  }
  if (x == ',') {
    while (x == ',' || x == ' ') x = Serial.read();
    bgndName[0] = x;
    for (int i = 1; i < 31; i++) {
      x = Serial.read();
      if (x < ' ' || x == ',') {
        bgndName[i] = 0;
        break;
      }
      bgndName[i] = x;
    }
  }
  while (Serial.available()) Serial.read(); // Flush the buffer
  Serial.print(lineName);
  Serial.print(F(" on "));
  Serial.println(bgndName);
}

void setup() {
  Serial.begin(115200);
  while (!Serial);
  getNames();           // To label the values
}

void clearHistograms() {
  for (int j = 0; j < HIST_SIZE; j++) {
    for (int i = 0; i < 3; i++) {
      lineHist[i][j] = 0;
      bgndHist[i][j] = 0;
      edgeHist[i][j] = 0;
    }
  }
}

void adjustAve(int16_t sensorNr, int16_t ave[], uint8_t hist[][HIST_SIZE]) {
  int16_t base = HIST_SIZE / 2;
  int16_t x = base;
  int16_t y = hist[sensorNr][x];
  int16_t j;

  for (int i = 0; i < base; i++) {
    j = base - i;
    if (hist[sensorNr][j] > y) {
      x = j;
      y =    hist[sensorNr][x] ;
    }
    j = base + i - 1;
    if (hist[sensorNr][j] > y) {
      x = j;
      y =    hist[sensorNr][x] ;
    }
  }
  int16_t delta = x - base;
  ave[sensorNr] += delta;
}

void adjustAverages() {
  for (int i = 0; i < 3; i++) {
    adjustAve(i, lineAve, lineHist);
    adjustAve(i, bgndAve, bgndHist);
    adjustAve(i, edgeAve, edgeHist);
  }
}

void cumulateData(int16_t sensorNr, int16_t sensorVal[], int16_t ave[], uint8_t hist[][HIST_SIZE]) {
  int16_t index   = (sensorVal[sensorNr] - ave[sensorNr]) + (HIST_SIZE / 2);
  if (index >= 0 && index < HIST_SIZE) {
    uint8_t x = hist[sensorNr][index];
    if (x < 255) hist[sensorNr][index] = x + 1;
  }
}

void collectSensorData() {
  int16_t val[3];
  IR3.readSensors(val);
  for (int16_t i = 0; i < 3; i++) {
    cumulateData(i, val, lineAve, lineHist);
    cumulateData(i, val, bgndAve, bgndHist);
    cumulateData(i, val, edgeAve, edgeHist);
  }
}

void calcAverages(int16_t ave[]) {
  int16_t n = 256;
  int16_t val[3];
  int32_t sum[3] = {0, 0, 0};
  for (int i = 0; i < n; i++) {
    delay(10);                  // Allocate time for "different readings"
    IR3.readSensors(val);
    for (int j = 0; j < 3; j++) {
      sum[j] += val[j];
    }
  }
  for (int j = 0; j < 3; j++) {
    ave[j] = sum[j] / n;
  }
}

void driveUntil(int16_t leftSpeed, int16_t rightSpeed, int16_t leftTarget, int16_t rightTarget) {
  int16_t leftBase = encoders.getCountsLeft();
  int16_t rightBase = encoders.getCountsRight();
  motors.setSpeeds(leftSpeed, rightSpeed);
  do {
    int16_t    leftDistance    = encoders.getCountsLeft() - leftBase;
    int16_t    rightDistance    = encoders.getCountsRight() - rightBase;

    if (leftSpeed < 0) {
      if (leftDistance <= leftTarget) {
        leftSpeed = 0;
        motors.setLeftSpeed(leftSpeed);
      }
    }
    if (leftSpeed > 0) {
      if (leftDistance >= leftTarget) {
        leftSpeed = 0;
        motors.setLeftSpeed(leftSpeed);
      }
    }

    if (rightSpeed < 0) {
      if (rightDistance <= rightTarget) {
        rightSpeed = 0;
        motors.setRightSpeed(rightSpeed);
      }
    }
    if (rightSpeed > 0) {
      if (rightDistance >= rightTarget) {
        rightSpeed = 0;
        motors.setRightSpeed(rightSpeed);
      }
    }
    delay(2);
    collectSensorData();
  } while (leftSpeed || rightSpeed);
}

void turnLeft(uint16_t target) {
  driveUntil(-100, 100, -target, target);
}

void turnRight(uint16_t target) {
  driveUntil(100, -100, target, -target);
}

void moveForward(uint16_t target) {
  driveUntil(75, 75, target, target);
}

void moveBackward(uint16_t target) {
  driveUntil(-75, -75, -target, -target);
}

void printCode(int16_t ave[], char *aveName, char *comment) {
  Serial.print(F("//\tconst int16_t "));
  Serial.print (aveName);
  Serial.print (F("[] = {"));
  Serial.print(ave[0]); Serial.print(',');
  Serial.print(ave[1]); Serial.print(',');
  Serial.print(ave[2]); Serial.print(F("};"));
  if (strlen(comment)) {
    Serial.print(F("  // "));
    Serial.print(comment);
  }
  Serial.println();
}

void calibrateLine() {       // Start all sensors on line
  calcAverages(lineAve);
  moveForward(250);
  calcAverages(bgndAve);
  clearHistograms();

  moveBackward(400);
  delay(100);
  moveForward(200);
  delay(100);
  moveBackward(200);
  delay(100);
  moveForward(150);
  delay(100);

  adjustAverages();

  printCode(lineAve, "lineAve", lineName);
  printCode(bgndAve, "bgndAve", bgndName);
}

void waitToStart() {
  lcd.gotoXY(0, 0);
  lcd.print("Measure ");
  delay(300);

}

void calibrateEdge() {
  lcd.clear();
  lcd.print(F("Press A:"));
  lcd.gotoXY(0, 1);
  lcd.print(F("Floor"));
  buttonA.waitForButton();
  waitToStart();
  calcAverages(bgndAve);
  buzzer.play("L16 cdegreg4");

  lcd.clear();
  lcd.print(F("Press B:"));
  lcd.gotoXY(0, 1);
  lcd.print(F("Boarder"));
  buttonB.waitForButton();
  waitToStart();
  calcAverages(lineAve);
  buzzer.play("L16 cdegreg4");

  lcd.clear();
  lcd.print(F("Press C:"));
  lcd.gotoXY(0, 1);
  lcd.print(F("Ovr Edge"));
  buttonC.waitForButton();
  waitToStart();
  calcAverages(edgeAve);
  buzzer.play("L16 cdegreg4");

  printCode(lineAve, "lineAve", "Dohyo Boarder");
  printCode(bgndAve, "bgndAve", "Dohyo Floor");
  printCode(edgeAve, "edgeAve", "Over Dohyo Edge");
}

void loop() {
  while (Serial.available()) Serial.read(); // Flush the buffer

  int16_t bVoltage = readBatteryMillivolts();
  Serial.print(F("Battery voltage = "));
  Serial.print(bVoltage);
  Serial.println(F(" mV"));
  Serial.println(F("Action: 1 = start on line, 2 = start on edge, 9 = enter line name"));
  while (!Serial.available());
  switch (Serial.read()) {
    case '1':
      if (bVoltage > 5000) {
        calibrateLine();
      } else {
        Serial.println(F("Too low voltage"));
      }
      break;
    case '2':
      calibrateEdge();
      break;
    case '9':
      getNames();
      break;
    default:
      Serial.println("Unknown action");
      break;
  }
}
