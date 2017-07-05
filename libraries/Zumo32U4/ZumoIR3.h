#ifndef ZUMOIR3_H
#define ZUMOIR3_H

#include <Arduino.h>

/*
  ZumoIR3.h - Library for using Pololu 3 channel QTR reflectance
    sensor array used in Zumo32U4 robot.  The hardware board has only 3
	sensors installed and 2 slots are empty.
	
	This library is based on the assumption that the sensor readings are
	externally calibrated for 
	 - background color, such as white paper, floor, or Dohyo ring
	 - line color, such as vinyl tape, marker, or Dohyo boarder
	 - edge color when sensors are over the Dohyo boarder

	There can be multiple calibratrion sets that are maintained in ZumoIR3.cpp
*/

class ZumoIR3 {
  public:
	ZumoIR3();
	static void		readSensors(uint16_t uncalibratedValues[]);
								// Line follower functions
	static uint16_t	readLeft();
	static uint16_t	readFront();
	static uint16_t	readRight();
								// Dohyo functions
	static uint16_t	readLeft2();
	static uint16_t	readFront2();
	static uint16_t	readRight2();
  private:
};
	
#endif
