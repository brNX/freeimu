/*
main.cpp - FreeIMU Stellaris port
Copyright (C) 2012, Ali Sabri Sanal, synerroronthoughts@ AT mail DOT com
*/

extern "C" {

#include <math.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_i2c.h"
#include "inc/hw_ints.h"
#include "driverlib/interrupt.h"
#include "driverlib/i2c.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/gpio.h"
#include "driverlib/eeprom.h"
#include "driverlib/fpu.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"
#include "utils/ustdlib.h"
#include <stdint.h>
//#include "calibration.h"

#define SYSTICKS_PER_SECOND     1000
  
unsigned long ulClockMS=0;

static unsigned long timingDelay = 0;
unsigned long milliSec = 0;

int decimalOf(float val)
{
	int retval = (val-(int)val)*100;
	return retval;
}

void delayMSec(unsigned long msec)
{
	MAP_SysCtlDelay(ulClockMS*msec);
}

void delayuSec(unsigned long usec)
{
	MAP_SysCtlDelay((ulClockMS/1000)*usec);
}

void SysTickHandler(void)
{

	milliSec++;

}

unsigned long millis(void)
{

	return milliSec;

}

void InitConsole(void)
{
	//
	// Enable GPIO port A which is used for UART0 pins.
	// TODO: change this to whichever GPIO port you are using.
	//
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

	//
	// Configure the pin muxing for UART0 functions on port A0 and A1.
	// This step is not necessary if your part does not support pin muxing.
	// TODO: change this to select the port/pin you are using.
	//
	//MAP_GPIOPinConfigure(GPIO_PA0_U0RX); //not needed on lm3s1776
	//MAP_GPIOPinConfigure(GPIO_PA1_U0TX); //not needed on lm3s1776

	//
	// Select the alternate (UART) function for these pins.
	// TODO: change this to select the port/pin you are using.
	//
	MAP_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

	//
	// Initialize the UART for console I/O.
	//
	UARTStdioInit(0);
}

void InitI2C(void)
{
	//
	// For this example I2C0 is used with PortB[3:2].  The actual port and
	// pins used may be different on your part, consult the data sheet for
	// more information.  GPIO port B needs to be enabled so these pins can
	// be used.
	//

	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

	//
	// The I2C0 peripheral must be enabled before use.
	//
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);

	//
	// Configure the pin muxing for I2C0 functions on port B2 and B3.
	// This step is not necessary if your part does not support pin muxing.
	//
	//MAP_GPIOPinConfigure(GPIO_PB2_I2C0SCL);
	//MAP_GPIOPinConfigure(GPIO_PB3_I2C0SDA);


	MAP_GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_2 | GPIO_PIN_3);
	
	MAP_I2CMasterInitExpClk(I2C0_MASTER_BASE,MAP_SysCtlClockGet(),true);  //false = 100khz , true = 400khz
}

//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, unsigned long ulLine)
{
}
#endif

}

#define F_CPU 50000000L

#include "MS561101BA/MS561101BA.h"
#include "HMC58X3/HMC58X3.h"
#include "MPU60X0/MPU60X0.h"
#include "FreeIMU/FreeIMU.h"
#include "FreeIMU/CommunicationUtils.h"

float q[4];
int16_t raw_values[9];
float ypr[3]; // yaw pitch roll
char str[256];
float val[9];
char cmd;
unsigned char tmp[sizeof(float)*6 + sizeof(int16_t)*6];
unsigned long pulData[13];

char serial_busy_wait()
{
  while(!UARTCharsAvail(UART0_BASE)) {
    ; // do nothing until ready
  }
  return (char)UARTCharGet(UART0_BASE);
}

int main(void)
{
	//
	// Set the clocking to run directly from the external crystal/oscillator.
	// TODO: The SYSCTL_XTAL_ value must be changed to match the value of the
	// crystal on your board.
	//

	//MAP_FPULazyStackingEnable(); //Not available on lm3s1776
	//MAP_FPUEnable();

	MAP_SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_12MHZ); //50MHz

	//
	// Enable peripherals to operate when CPU is in sleep.
	//
	MAP_SysCtlPeripheralClockGating(true);

	//MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_EEPROM0); //No eeprom available , maybe use Using Stellaris MCUs Internal Flash Memory to Emulate EEPROM (AN01267)
	//EEPROMInit();
	
	// Get the current processor clock frequency.
	ulClockMS = MAP_SysCtlClockGet() / (3 * 1000);

	InitConsole();

	InitI2C();

	//
	// Configure SysTick to occur 100 times per second, to use as a time
	// reference.  Enable SysTick to generate interrupts.
	//
	MAP_SysTickPeriodSet(MAP_SysCtlClockGet() / SYSTICKS_PER_SECOND);
	MAP_SysTickIntEnable();
	MAP_SysTickEnable();


	MAP_I2CMasterInitExpClk(I2C0_MASTER_BASE, SysCtlClockGet(), true);

	FreeIMU my3IMU=FreeIMU();

	delayMSec(100);

	my3IMU.init(true); // the parameter enable or disable fast mode

	delayMSec(5);

//	UARTprintf("starting\n");

	while (1) {
	  if(UARTCharsAvail(UART0_BASE)) {
	    cmd = (char)UARTCharGet(UART0_BASE);
	    if(cmd=='v') {
	      UARTprintf("FreeIMU library by %s, FREQ:%s, LIB_VERSION: %s, IMU: %s\n", FREEIMU_DEVELOPER, FREEIMU_FREQ, FREEIMU_LIB_VERSION, FREEIMU_ID);
	    }
	    else if(cmd=='r') {
	      uint8_t count = serial_busy_wait();
	      for(uint8_t i=0; i<count; i++) {
	        my3IMU.getRawValues(raw_values);
	        UARTprintf("%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,\n", raw_values[0], raw_values[1], raw_values[2], raw_values[3], raw_values[4], raw_values[5], raw_values[6], raw_values[7], raw_values[8], raw_values[9], raw_values[10]);
	      }
	    }
	    else if(cmd=='b') {
	      uint8_t count = serial_busy_wait();
	      for(uint8_t i=0; i<count; i++) {
	        #if HAS_ITG3200()
	          my3IMU.acc.readAccel(&raw_values[0], &raw_values[1], &raw_values[2]);
	          my3IMU.gyro.readGyroRaw(&raw_values[3], &raw_values[4], &raw_values[5]);
	        #else // MPU6050
	          my3IMU.accgyro.getMotion6(&raw_values[0], &raw_values[1], &raw_values[2], &raw_values[3], &raw_values[4], &raw_values[5]);
	        #endif
	        writeArr(raw_values, 6, sizeof(int16_t)); // writes accelerometer and gyro values
	        #if IS_9DOM()
	          my3IMU.magn.getValues(&raw_values[0], &raw_values[1], &raw_values[2]);
	          writeArr(raw_values, 3, sizeof(int16_t));
	        #endif
	        UARTprintf("\n");
	      }
	    }
	    else if(cmd == 'q') {
	      uint8_t count = serial_busy_wait();
	      for(uint8_t i=0; i<count; i++) {
	        my3IMU.getQ(q);
	        serialPrintFloatArr(q, 4);
	        UARTprintf("\n");
	      }
	    }
	    #ifndef CALIBRATION_H
	    else if(cmd == 'c') {

	    /*	const uint8_t eepromsize = sizeof(float) * 6 + sizeof(int16_t) * 6;
	    	int i=0, j=0;

	      while(i<eepromsize) {
	      	tmp[i++] = (unsigned char)UARTCharGet(UART0_BASE); // wait until all calibration data are received
	      }
        pulData[0] = FREEIMU_EEPROM_SIGNATURE;

	      for (i=1; i<6+1; i++) {
	        memcpy((void*)&pulData[i], (void*)&tmp[j], sizeof(int16_t));
	        j += sizeof(int16_t);
	      }

	      for (i=7; i<13; i++) {
	        memcpy((void*)&pulData[i], (void*)&tmp[j], sizeof(float));
	        j += sizeof(float);
	      }

	    	EEPROMProgram(pulData, 0x0, sizeof(pulData));

	      my3IMU.calLoad(); // reload calibration*/

	    } else if (cmd=='x') {
	    /*	pulData[0] = 0;
	    	EEPROMProgram(pulData, 0x0, sizeof(unsigned long));
	      my3IMU.calLoad(); // reload calibration*/

	    }
	    #endif
	    else if(cmd == 'C') { // check calibration values
	      UARTprintf("acc offset: %d,%d,%d\n", my3IMU.acc_off_x, my3IMU.acc_off_y, my3IMU.acc_off_z);
	      UARTprintf("magn offset: %d,%d,%d\n", my3IMU.magn_off_x, my3IMU.magn_off_y, my3IMU.magn_off_z);
	      UARTprintf("acc scale: %d.%02d,%d.%02d,%d.%02d\n", (int)my3IMU.acc_scale_x, decimalOf(my3IMU.acc_scale_x),
	      																										(int)my3IMU.acc_scale_y, decimalOf(my3IMU.acc_scale_y),
	      																										(int)my3IMU.acc_scale_z, decimalOf(my3IMU.acc_scale_z));
	      UARTprintf("magn scale: %d.%02d,%d.%02d,%d.%02d\n", (int)my3IMU.magn_scale_x, decimalOf(my3IMU.magn_scale_x),
	      																										(int)my3IMU.magn_scale_y, decimalOf(my3IMU.magn_scale_y),
	      																										(int)my3IMU.magn_scale_z, decimalOf(my3IMU.magn_scale_z));
	    }
	    else if(cmd == 'd') { // debugging outputs
	      while(1) {
	        my3IMU.getRawValues(raw_values);
	        UARTprintf("%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,\n", raw_values[0], raw_values[1], raw_values[2], raw_values[3], raw_values[4], raw_values[5], raw_values[6], raw_values[7], raw_values[8], raw_values[9], raw_values[10]);
	        my3IMU.getQ(q);
	        serialPrintFloatArr(q, 4);
	        UARTprintf("\n");
	        my3IMU.getYawPitchRoll(ypr);
	        UARTprintf("Yaw: %d.%02d Pitch: %d.%02d Roll: %d.%02d\n", (int)ypr[0], decimalOf(ypr[0]),
	        																													(int)ypr[1], decimalOf(ypr[1]),
	        																													(int)ypr[2], decimalOf(ypr[2]));
	        break;
	      }
	    }
	  }
	}

}

