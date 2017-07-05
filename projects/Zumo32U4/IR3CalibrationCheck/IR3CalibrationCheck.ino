/**
 * Created by Olavi Kamppari on 7/4/2017.
 */

/**
 * Used to test the line sensor calibration values stored in ZumoIR3.cpp
 */
 
#include <Zumo32U4.h>
#include <ZumoIR3.h>

Zumo32U4LCD         lcd;
ZumoIR3             IR3;

void setup() {
}

void loop() {
  lcd.clear();
  lcd.gotoXY(3, 0);
  lcd.print(IR3.readFront2());
  lcd.gotoXY(0, 1);
  lcd.print(IR3.readLeft2());
  lcd.gotoXY(5, 1);
  lcd.print(IR3.readRight2());
  delay(20);
}

