/*****************************************************************************
 * serial_id_ds2411.h
 ******************************************************************************/
 
 
#ifndef _SERIAL_ID_DS2411_H_
#define _SERIAL_ID_DS2411_H_

#include "gpio.h"

#define ECLAVE_SERIAL_ID0	AVR32_PIN_PB19
#define ECLAVE_SERIAL_ID1	AVR32_PIN_PB20
#define ECLAVE_SERIAL_ID2	AVR32_PIN_PB21
#define ECLAVE_SERIAL_ID3	AVR32_PIN_PB22
#define ECLAVE_SERIAL_ID4	AVR32_PIN_PB23


void SetSpeed(int standard);
int OWTouchReset(unsigned char idx);
void OWWriteByte(unsigned char idx, int data);
int OWReadByte(unsigned char idx);
void gpio_input(unsigned char idx);

#define EC_CPU_CLOCK_FREQ 100000000UL

#endif  // _SERIAL_ID_DS2411_H_
