extern "C" {

#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"
#include <math.h>
#include <stdint.h>
}
#include "CommunicationUtils.h"

void serialPrintFloatArr(float * arr, int length)
{
  for(int i=0; i<length; i++) {
    serialFloatPrint(arr[i]);
		UARTprintf(",");
  }
}


void serialFloatPrint(float f)
{
  uint8_t* b = (uint8_t*) &f;

  for (int i=0; i<4; i++) {
    
    uint8_t b1 = (b[i] >> 4) & 0x0f;
    uint8_t b2 = (b[i] & 0x0f);
    
    char c1 = (b1 < 10) ? ('0' + b1) : 'A' + b1 - 10;
    char c2 = (b2 < 10) ? ('0' + b2) : 'A' + b2 - 10;
    
    UARTprintf("%c", c1);
    UARTprintf("%c", c2);
  }
}


void writeArr(void* varr, uint8_t arr_length, uint8_t type_uint8_ts)
{
  uint8_t* arr = (uint8_t*) varr;

  for (uint8_t i=0; i<arr_length; i++) {
    writeVar(&arr[i * type_uint8_ts], type_uint8_ts);
  }
}


// thanks to Francesco Ferrara and the Simplo project for the following code!
void writeVar(void * val, uint8_t type_uint8_ts)
{
  char* addr=(char*)(val);

  for (int i=0; i<type_uint8_ts; i++)
  	UARTCharPut(UART0_BASE, addr[i]);
}
