/**
 *  Created by Olavi Kamppari on 6/26/2017.
 *
 */
 
/*
  ZumoIR3.cpp - Library for using Pololu 3 channel QTR reflectance
    sensor array used in Zumo32U4 robot.  The hardware board has only 3
	sensors installed and 2 slots are empty.
	
	This library is based on the assumption that the sensor readings are
	externally calibrated for 
	 - background color, such as white paper, floor, or Dohyo ring
	 - line color, such as vinyl tape, marker, or Dohyo boarder
	 - edge color when sensors are over the Dohyo boarder

	There are multiple calibratrion sets that are created by ZumoIR3Calibrate
	When used in new environments, new calibrations are required
	When changing environment, the right calibrations have to be selected
	The unused calibrations are commented out
*/

/*
*	readSensors method returns the 10 bit ADC values from the 3 sensors
*	Nominally the values are from 0 to 1023
*	In practice the values are from 100 to 980
*
*	readXXX method returns value from 0 to 100
*		0: sensor is reading background value
*		1-99: sensor is reading a value between background and line
*		100: sensor is reading a line value
*
*	readXXX2 method returns a value from 0 to 200
*		0: sensor is reading Dohyo background
*		1..99: sensor is reading Dohyo background or boarder
*		100: sensor is reading Dohyo boarder
*		101..199: sensor is reading Dohyo boarder or over edge
*		200: sensor is over the edge
*
*/

#include <ZumoIR3.h>

// Background sets
	const int16_t bgndAve[] = {644,410,678};  // Floor
//	const int16_t bgndAve[] = {715,566,777};  // Dohyo Floor

// Line sets
//	const int16_t lineAve[] = {235,195,495};  // White Tape
	const int16_t lineAve[] = {948,909,966};  // Black Tape
//	const int16_t lineAve[] = {919,866,922};  // Dohyo Boarder

// Edge sets
	const int16_t edgeAve[] = {935,890,933};  // Over Dohyo Edge

// The sensor names are shown at the bottom of the Zumo robot
#define ZUMOIR_LFT A0
#define ZUMOIR_FRONT A3
#define ZUMOIR_RGT A11
#define ZUMOIR_LED 11

ZumoIR3::ZumoIR3() {					// Initialization
  pinMode(ZUMOIR_LFT, INPUT);
  pinMode(ZUMOIR_FRONT, INPUT);
  pinMode(ZUMOIR_RGT, INPUT);

  digitalWrite(ZUMOIR_LFT, HIGH);
  digitalWrite(ZUMOIR_FRONT, HIGH);
  digitalWrite(ZUMOIR_RGT, HIGH);

  pinMode(ZUMOIR_LED, OUTPUT);          // Light for the line sensrors
  digitalWrite(ZUMOIR_LED, HIGH);       // Turn light on	
}

static void		ZumoIR3::readSensors(uint16_t uncalibratedValues[]) {
  uncalibratedValues[0]    = analogRead(ZUMOIR_LFT);
  uncalibratedValues[1]    = analogRead(ZUMOIR_FRONT);
  uncalibratedValues[2]    = analogRead(ZUMOIR_RGT);	
}

static uint16_t interpolate(int16_t x, int16_t x0, int16_t x1) {
	int32_t dx1 = x1 - x0;
	int32_t dx  = 100L * (x - x0);
	int32_t dy = dx / dx1;
	if (dy < 0) return 0;
	if (dy >100) return 100;
	return dy;
}

static uint16_t	ZumoIR3::readLeft() {
	return interpolate(analogRead(ZUMOIR_LFT),bgndAve[0],lineAve[0]);
}

static uint16_t	ZumoIR3::readFront() {
	return interpolate(analogRead(ZUMOIR_FRONT),bgndAve[1],lineAve[1]);
}


static uint16_t	ZumoIR3::readRight() {
	return interpolate(analogRead(ZUMOIR_RGT),bgndAve[2],lineAve[2]);
}

static uint16_t interpolate2(int16_t x, int16_t x0, int16_t x1, int16_t x2) {
	int32_t dx1 = x1 - x0;
	int32_t dx  = 100L * (x - x0);
	int32_t dy = dx / dx1;
	if (dy < 0) return 0;
	if (dy <= 100) return dy; // Inside the Dohyo
	int32_t dx2 = x2 - x1;
	dx  = 100L * (x - x1);
	dy = dx / dx1;
	if (dy < 0) return 100;
	if (dy > 100) return 200;
	return 100 + dy;
}


static uint16_t	ZumoIR3::readLeft2() {
	return interpolate2(analogRead(ZUMOIR_LFT),bgndAve[0],lineAve[0],edgeAve[0]);
}

static uint16_t	ZumoIR3::readFront2() {
	return interpolate2(analogRead(ZUMOIR_FRONT),bgndAve[1],lineAve[1],edgeAve[1]);
}


static uint16_t	ZumoIR3::readRight2() {
	return interpolate2(analogRead(ZUMOIR_RGT),bgndAve[2],lineAve[2],edgeAve[2]);
}
