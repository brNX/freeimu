/*
MS5611-01BA.cpp - Interfaces a Measurement Specialities MS5611-01BA with Arduino
See http://www.meas-spec.com/downloads/MS5611-01BA01.pdf for the device datasheet

Copyright (C) 2011 Fabio Varesano <fvaresano@yahoo.it>

Development of this code has been supported by the Department of Computer Science,
Universita' degli Studi di Torino, Italy within the Piemonte Project
http://www.piemonte.di.unito.it/


This program is free software: you can redistribute it and/or modify
it under the terms of the version 3 GNU General Public License as
published by the Free Software Foundation.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

*/
extern "C" {

#include "I2Cdev/i2cutil.h"
#include "utils/uartstdio.h"
#include <math.h>
#include <stdint.h>
#include <string.h>

extern void delayMSec(unsigned long msec);
extern unsigned long timingDelay;
extern unsigned long millis(void);
}

#include "MS561101BA.h"
#define CONVERSION_TIME 10 // conversion time in miliseconds

/*
void printLongLong(uint64_t n, uint8_t base) {
  do {
    uint64_t m = n;
    n /= base;
    char c = m - base * n;
    c = c < 10 ? c + '0' : c + 'A' - 10;
    Serial.print(c);
  } while(n);
  Serial.println();
}
*/


MS561101BA::MS561101BA() {
}

void MS561101BA::init(uint8_t address)
{
  _addr =  address;
  
  reset(); // reset the device to populate its internal PROM registers
  delayMSec(1000); // some safety time
  readPROM(); // reads the PROM into object variables for later use
}

float MS561101BA::getPressure(uint8_t OSR)
{
  // see datasheet page 7 for formulas
  
  uint32_t rawPress = rawPressure(OSR);
  //UARTprintf("rawpres=%d\n", (int)rawPress);
  if(rawPress == NULL) {
    return NULL;
  }
  
  int32_t dT = getDeltaTemp(OSR);
  if(dT == NULL) {
    return NULL;
  }
  
  int64_t off  = ((uint32_t)_C[1] <<16) + (((int64_t)dT * _C[3]) >> 7);
  int64_t sens = ((uint32_t)_C[0] <<15) + (((int64_t)dT * _C[2]) >> 8);
  return ((( (rawPress * sens ) >> 21) - off) >> 15) / 100.0;
}

float MS561101BA::getTemperature(uint8_t OSR)
{
  // see datasheet page 7 for formulas
  int64_t dT = getDeltaTemp(OSR);
  
  if(dT != NULL) {
    return (2000 + ((dT * _C[5]) >> 23)) / 100.0;
  }
  else {
    return NULL;
  }
}

int32_t MS561101BA::getDeltaTemp(uint8_t OSR)
{
  uint32_t rawTemp = rawTemperature(OSR);
  if(rawTemp != NULL) {
    return (int32_t)(rawTemp - ((uint32_t)_C[4] << 8));
  }
  else {
    return NULL;
  }
}

//TODO: avoid duplicated code between rawPressure and rawTemperature methods
//TODO: possible race condition between readings.. serious headache doing this.. help appreciated!

uint32_t MS561101BA::rawPressure(uint8_t OSR)
{
  unsigned long now = millis();

  if(lastPresConv != 0 && (now - lastPresConv) >= CONVERSION_TIME) {
    lastPresConv = 0;
    pressCache = getConversion(MS561101BA_D1 + OSR);
  }
  else {
    if(lastPresConv == 0 && lastTempConv == 0) {
      startConversion(MS561101BA_D1 + OSR);
      lastPresConv = now;
    }
  }
  return pressCache;
}

uint32_t MS561101BA::rawTemperature(uint8_t OSR)
{
  unsigned long now = millis();

  if(lastTempConv != 0 && (now - lastTempConv) >= CONVERSION_TIME) {
    lastTempConv = 0;
    tempCache = getConversion(MS561101BA_D2 + OSR);
  }
  else {
    if(lastTempConv == 0 && lastPresConv == 0) { // no conversions in progress
      startConversion(MS561101BA_D2 + OSR);
      lastTempConv = now;
    }
  }
  return tempCache;
}


// see page 11 of the datasheet
void MS561101BA::startConversion(uint8_t command)
{
	unsigned char tx[1];
	tx[0] = command;

	i2cWrite(_addr, tx, 1);
}

uint32_t MS561101BA::getConversion(uint8_t command)
{
  union {uint32_t val; uint8_t raw[4]; } conversion = {0};
  
	unsigned char tx[1];
	unsigned char rx[3];
	tx[0] = 0;

	i2cWrite(_addr, tx, 1);
	if (!i2cRead(_addr, rx, 3)) {

	  conversion.raw[2] = rx[0];
    conversion.raw[1] = rx[1];
    conversion.raw[0] = rx[2];
  }
  else {
    conversion.val = -1;
  }
  
  return conversion.val;
}


/**
 * Reads factory calibration and store it into object variables.
*/
int MS561101BA::readPROM()
{
	unsigned char tx[1];
	unsigned char rx[3];

  for (int i=0;i<MS561101BA_PROM_REG_COUNT;i++) {

  	tx[0] = MS561101BA_PROM_BASE_ADDR + (i * MS561101BA_PROM_REG_SIZE);
  	i2cWrite(_addr, tx, 1);

  	if (!i2cRead(_addr, rx, 2)) {
      _C[i] = (rx[0] << 8) | rx[1];
      
      //DEBUG_PRINT(_C[i]);
    }
    else {
      return -1; // error reading the PROM or communicating with the device
    }
  }
  return 0;
}


/**
 * Send a reset command to the device. With the reset command the device
 * populates its internal registers with the values read from the PROM.
*/
void MS561101BA::reset()
{

	unsigned char tx[1];
	tx[0] = MS561101BA_RESET;

	i2cWrite(_addr, tx, 1);
	delayMSec(3);
}



