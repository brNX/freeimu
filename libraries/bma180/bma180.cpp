
extern "C" {

#include "I2Cdev/i2cutil.h"
#include "utils/uartstdio.h"
#include <math.h>

extern void delayMSec(unsigned long msec);
}

#include "bma180.h"


BMA180::BMA180()
{
	gSense=G2;
}

void BMA180::setAddress(int adr)
{
	address=(unsigned char) adr;
}

int temp;

void BMA180::readAccel(int * buff) {
	readAccel(&buff[0], &buff[1], &buff[2]);
}

void BMA180::readAccel(int * x, int * y, int * z)
{
	unsigned char tx[1];
	unsigned char rx[7];
	tx[0] = 0x02;

	i2cWrite(address, tx, 1);
	i2cRead(address, rx, 7);

	int lsb = rx[0]>>2;
	int msb = rx[1];
	*x=(msb<<6)+lsb;
	if ((*x)&0x2000) (*x)|=0xc000; // set full 2 complement for neg values
	lsb = rx[2]>>2;
	msb = rx[3];
	*y=(msb<<6)+lsb;
	if ((*y)&0x2000) (*y)|=0xc000;
	lsb = rx[4]>>2;
	msb = rx[5];
	*z=(msb<<6)+lsb;
	if ((*z)&0x2000) (*z)|=0xc000;
	temp = rx[6];
	if (temp&0x80) temp|=0xff00;
}

float BMA180::getGSense()
{
	switch(gSense)
	{
	case G1: return 1.0;
	case G15: return 1.5;
	case G2: return 2.0;
	case G3: return 3.0;
	case G4: return 4.0;
	case G8: return 8.0;
	case G16: return 16.0;
	}
}

float BMA180::getXValFloat()
{
	// normalize (if x is maximum (8191) and GSENSE=1.0 then 1.0
	return (float)x/8191.0*getGSense();
}
float BMA180::getYValFloat()
{
	// normalize (if x is maximum (8191) and GSENSE=1.0 then 1.0
	return (float)y/8191.0*getGSense();
}
float BMA180::getZValFloat()
{
	// normalize (if x is maximum (8191) and GSENSE=1.0 then 1.0
	return (float)z/8191.0*getGSense();
}

int BMA180::getRegValue(int adr)
{
	unsigned char tx[1];
	unsigned char rx[1];
	tx[0] = adr;

	i2cWrite(address, tx, 1);
	int result = i2cRead(address, rx, 1);

	checkResult(result);
	return (int)rx[0];
}

void BMA180::setRegValue(int regAdr, int val, int maskPreserve)
{
	unsigned char tx[2];
	tx[0] = regAdr;

	int preserve=getRegValue(regAdr);
	int orgval=preserve & maskPreserve;
	tx[1] = orgval|val;

	int result = i2cWrite(address, tx, 2);
	checkResult(result);

}

void BMA180::setGSensitivty(GSENSITIVITY maxg) //1, 1.5 2 3 4 8 16
{
	setRegValue(0x35,maxg<<1,0xF1);
}

void BMA180::SetFilter(FILTER f) // 10,20,40,75,150,300,600,1200, HP 1HZ,BP 0.2-300, higher values not authorized
{
	setRegValue(0x20,f<<4,0x0F);
}

void BMA180::SetISRMode() // you must provide a ISR function on the pin selected (pin 2 or 3,. so INT0 or INT1)
{
	setRegValue(0x21,2,0xFD);
}

void BMA180::SoftReset() // all values will be default
{
	setRegValue(0x10,0xB6,0);
	delayMSec(100);
}

void BMA180::SetSMPSkip()
{
	setRegValue(0x35, 1, 0xFE);
}

int BMA180::getIDs(int *id, int *version)
{      
	unsigned char tx[1];
	unsigned char rx[2];
	tx[0] = 0;

	i2cWrite(address, tx, 1);
	int result = i2cRead(address, rx, 2);

	if (checkResult(result)) {
		*id = (int)rx[0];
		*version = (int)rx[1];
	} else {
		*id = *version = -1;
	}
	return *id!=-1;
}


void BMA180::enableWrite()
{
	//ctrl_reg1 register set ee_w bit to enable writing to regs.
	setRegValue(0x0D,0x10,~0x10);
	delayMSec(10);
}


void BMA180::disableWrite()
{
	setRegValue(0x0D,0x0,~0x10);
	delayMSec(10);
}

bool BMA180::checkResult(int result)
{
	return (!result);
}    


