/*****************************************************************************
 * serial_id_ds2411.c
 ******************************************************************************/

#include "gpio.h"
#include "cycle_counter.h"
#include "serial_id_ds2411.h"

#define EC_ONE_MICROSECOND 8

unsigned char io_pin(unsigned char idx);
unsigned char io_pin(unsigned char idx)
{
	switch (idx)
	{
		case 0:
			return ECLAVE_SERIAL_ID0;
			break;
		case 1:
			return ECLAVE_SERIAL_ID1;
			break;
		case 2:
			return ECLAVE_SERIAL_ID2;
			break;
		case 3:
			return ECLAVE_SERIAL_ID3;
			break;
		case 4:
			return ECLAVE_SERIAL_ID4;
			break;
		default: 
			return 0; //TODO: return a better error code here
			break;
	}
}

void drive_DQ_low(unsigned char idx);
void drive_DQ_low(unsigned char idx)
{
	unsigned char ioPin;
	int32_t ioFlags;
	
	ioPin = io_pin(idx);
	
	ioFlags = (GPIO_DIR_OUTPUT); //14may15 experiment
	gpio_configure_pin(ioPin, ioFlags); //14may15 experiment

	gpio_set_pin_low(ioPin);

}

void release_the_bus(unsigned char idx);
void release_the_bus(unsigned char idx)
{
	unsigned char ioPin;
	int32_t ioFlags;
	
	ioPin = io_pin(idx);
	
//14may15 experiment	gpio_set_pin_high(ioPin);
	
	ioFlags = (GPIO_DIR_INPUT); //14may15 experiment
	gpio_configure_pin(ioPin, ioFlags); //14may15 experiment

}

void gpio_input(unsigned char idx) //14may15 experiment
{
	uint32_t ioFlags;
	unsigned char ioPin;
		
	ioPin = io_pin(idx);
		
	ioFlags = (GPIO_DIR_INPUT);
	gpio_configure_pin(ioPin, ioFlags);

}

unsigned char sample_line(unsigned char idx);
unsigned char sample_line(unsigned char idx)
{
//14may15 experiment		uint32_t ioFlags;
		unsigned char retVal, ioPin;
		
		ioPin = io_pin(idx);
		
//14may15 experiment		ioFlags = (GPIO_DIR_INPUT);
//14may15 experiment		gpio_configure_pin(ioPin, ioFlags);

		retVal = gpio_get_pin_value(ioPin);

//14may15 experiment		ioFlags = (GPIO_DIR_OUTPUT);
//14may15 experiment		gpio_configure_pin(ioPin, ioFlags);

		return retVal;
}


// 'tick' values
int		A, B, C, D, E, F, G, H, I, J;

//-----------------------------------------------------------------------------
// Set the 1-Wire timing to 'standard' (standard=1) or 'overdrive' (standard=0).
//
void SetSpeed(int standard)
{
	// Adjust tick values depending on speed
	if (standard)
	{
		// Standard Speed
//14may15 we can't seem to control this tightly		A = 6; //us
		A = 1; //should be 6 14may15
		B = 64;
		C = 60;
//14may15 we can't seem to control this tightly		D = 10;
//14may15 we can't seem to control this tightly		E = 9;
		D = 1; //should be 10 14may15
		E = 3; //should be 9 14may15
		F = 55;
		G = 0;
		H = 480;
		I = 70;
		J = 410;
	}
	else
	{
		// Overdrive Speed
		A = 1.5;
		B = 7.5;
		C = 7.5;
		D = 2.5;
		E = 0.75;
		F = 7;
		G = 2.5;
		H = 70;
		I = 8.5;
		J = 40;
	}
}

//-----------------------------------------------------------------------------
// Generate a 1-Wire reset, return 1 if no presence detect was found,
// return 0 otherwise.
// (NOTE: Does not handle alarm presence from DS2404/DS1994)
//
int OWTouchReset(unsigned char idx)
{
	int result;

	cpu_delay_us(A, 8000000);
	drive_DQ_low(idx);
	cpu_delay_us(H, 8000000);	//tRSTL (reset low) 480-640us
	release_the_bus(idx);
	
	gpio_input(idx); //14may15 experiment

	
	cpu_delay_us(I, 8000000);	//tMSP (presence detect sample) 60-75us
	result = sample_line(idx);
	
	gpio_input(idx); //14may15 experiement

	cpu_delay_us(J, 8000000); // Complete the reset sequence recovery 5-??us (no max?)
	return result; // Return sample presence pulse result
}


void drive_DQ_low_and_release_the_bus(unsigned char idx)
{
	unsigned char ioPin;
	int32_t ioFlagsOutput, ioFlagsInput;
	
	ioPin = io_pin(idx);
	
	ioFlagsInput = (GPIO_DIR_INPUT);
	ioFlagsOutput = (GPIO_DIR_OUTPUT); //14may15 experiment

	gpio_configure_pin(ioPin, ioFlagsOutput); //14may15 experiment

	gpio_set_pin_low(ioPin);

	gpio_configure_pin(ioPin, ioFlagsInput); //14may15 experiment
	
}

//-----------------------------------------------------------------------------
// Send a 1-Wire write bit. Provide 10us recovery time.
//
void OWWriteBit(unsigned char idx, int bit);
void OWWriteBit(unsigned char idx, int bit)
{
	if (bit)
	{
		// Write '1' bit
		drive_DQ_low_and_release_the_bus(idx);
#if 0
		drive_DQ_low(idx);
//14may15 take this out entirely, we can't seem to control this precisely enough		cpu_delay_us(A, 8000000);	//tW1L 5-15us
		release_the_bus(idx);
#endif
		cpu_delay_us(B, 8000000);	// Complete the time slot and 10us recovery tSLOT 65-??us (no max)
	}
	else
	{
		// Write '0' bit
		drive_DQ_low(idx);
		cpu_delay_us(C, 8000000);	//tW0L 60-120us
		release_the_bus(idx);
		cpu_delay_us(D, 8000000);	//tREC 5-??us
	}
}

//-----------------------------------------------------------------------------
// Read a bit from the 1-Wire bus and return it. Provide 10us recovery time.
//
int OWReadBit(unsigned char idx);
int OWReadBit(unsigned char idx)
{
	int result;

#if 0
	drive_DQ_low(idx);
//14may15 take this out entirely, we can't seem to control this precisely enough	cpu_delay_us(A, 8000000);	//tRL 5-15us
	release_the_bus(idx);
#endif
	drive_DQ_low_and_release_the_bus(idx);
	
//14may15 take this out, too tight	cpu_delay_us(E, 8000000);	//tMSR 5-15us
	result = sample_line(idx);
	cpu_delay_us(F, 8000000); // Complete the time slot and 10us recovery tREC 5+us

	return result;
}

//-----------------------------------------------------------------------------
// Write 1-Wire data byte
//
void OWWriteByte(unsigned char idx, int data)
{
	int loop;

	// Loop to write each bit in the byte, LS-bit first
	for (loop = 0; loop < 8; loop++)
	{
		OWWriteBit(idx, data & 0x01);

		// shift the data byte for the next bit
		data >>= 1;
	}
}

//-----------------------------------------------------------------------------
// Read 1-Wire data byte and return it
//
int OWReadByte(unsigned char idx)
{
	int loop, result=0;

	for (loop = 0; loop < 8; loop++)
	{
		// shift the result to get it ready for the next bit
		result >>= 1;

		// if result is one, then set MS bit
		if (OWReadBit(idx))
		result |= 0x80;
	}
	return result;
}

//-----------------------------------------------------------------------------
// Write a 1-Wire data byte and return the sampled result.
//
int OWTouchByte(unsigned char idx, int data);
int OWTouchByte(unsigned char idx, int data)
{
	int loop, result=0;

	for (loop = 0; loop < 8; loop++)
	{
		// shift the result to get it ready for the next bit
		result >>= 1;

		// If sending a '1' then read a bit else write a '0'
		if (data & 0x01)
		{
			if (OWReadBit(idx))
			result |= 0x80;
		}
		else
		OWWriteBit(idx, 0);

		// shift the data byte for the next bit
		data >>= 1;
	}
	return result;
}

//-----------------------------------------------------------------------------
// Write a block 1-Wire data bytes and return the sampled result in the same
// buffer.
//
void OWBlock(unsigned char idx, unsigned char *data, int data_len);
void OWBlock(unsigned char idx, unsigned char *data, int data_len)
{
	int loop;

	for (loop = 0; loop < data_len; loop++)
	{
		data[loop] = OWTouchByte(idx, data[loop]);
	}
}

//-----------------------------------------------------------------------------
// Set all devices on 1-Wire to overdrive speed. Return '1' if at least one
// overdrive capable device is detected.
//
int OWOverdriveSkip(unsigned char idx, unsigned char *data, int data_len);
int OWOverdriveSkip(unsigned char idx, unsigned char *data, int data_len)
{
	// set the speed to 'standard'
	SetSpeed(1);

	// reset all devices
	if (OWTouchReset(idx)) // Reset the 1-Wire bus
	return 0; // Return if no devices found

	// overdrive skip command
	OWWriteByte(idx, 0x3C);

	// set the speed to 'overdrive'
	SetSpeed(0);

	// do a 1-Wire reset in 'overdrive' and return presence result
	return OWTouchReset(idx);
}

//-----------------------------------------------------------------------------
// Read and return the page data and SHA-1 message authentication code (MAC)
// from a DS2432.
//
int ReadPageMAC(unsigned char idx, int page, unsigned char *page_data, unsigned char *mac);
int ReadPageMAC(unsigned char idx, int page, unsigned char *page_data, unsigned char *mac)
{
	int i;
	unsigned short data_crc16, mac_crc16;

	// set the speed to 'standard'
	SetSpeed(1);

	// select the device
	if (OWTouchReset(idx)) // Reset the 1-Wire bus
	return 0; // Return if no devices found

	OWWriteByte(idx, 0xCC); // Send Skip ROM command to select single device

	// read the page
	OWWriteByte(idx, 0xA5); // Read Authentication command
	OWWriteByte(idx, (page << 5) & 0xFF); // TA1
	OWWriteByte(idx, 0); // TA2 (always zero for DS2432)

	// read the page data
	for (i = 0; i < 32; i++)
	page_data[i] = OWReadByte(idx);
	OWWriteByte(idx, 0xFF);

	// read the CRC16 of command, address, and data
	data_crc16 = OWReadByte(idx);
	data_crc16 |= (OWReadByte(idx) << 8);

	// delay 2ms for the device MAC computation
	// read the MAC
	for (i = 0; i < 20; i++)
	mac[i] = OWReadByte(idx);

	// read CRC16 of the MAC
	mac_crc16 = OWReadByte(idx);
	mac_crc16 |= (OWReadByte(idx) << 8);

	// check CRC16...
	return 1;
}