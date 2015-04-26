/*****************************************************************************
 * serial_id_ds2411.c
 ******************************************************************************/

#include "gpio.h"
#include "cycle_counter.h"
#include "serial_id_ds2411.h"

#define SS_ONE_MICROSECOND 8


unsigned char io_pin(unsigned char idx)
{
	switch (idx)
	{
		case 0:
			return SEALSHIELD_SERIAL_ID0;
			break;
		case 1:
			return SEALSHIELD_SERIAL_ID1;
			break;
		case 2:
			return SEALSHIELD_SERIAL_ID2;
			break;
		case 3:
			return SEALSHIELD_SERIAL_ID3;
			break;
		case 4:
			return SEALSHIELD_SERIAL_ID4;
			break;
	}
}


void drive_DQ_low(unsigned char idx)
{
	unsigned char ioPin;
	
	ioPin = io_pin(idx);
	
	gpio_set_pin_low(ioPin);
}

void release_the_bus(unsigned char idx)
{
	unsigned char ioPin;
	
	ioPin = io_pin(idx);
	
	gpio_set_pin_high(ioPin);
	
}

unsigned char sample_line(unsigned char idx)
{
		uint32_t ioFlags;
		unsigned char retVal, ioPin;
		
		ioPin = io_pin(idx);
		
		ioFlags = (GPIO_DIR_INPUT);
		gpio_configure_pin(ioPin, ioFlags);

		retVal = gpio_get_pin_value(ioPin);

		ioFlags = (GPIO_DIR_OUTPUT);
		gpio_configure_pin(ioPin, ioFlags);

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
		A = 6; //us
		B = 64;
		C = 60;
		D = 10;
		E = 9;
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
	cpu_delay_us(H, 8000000);
	release_the_bus(idx);
	cpu_delay_us(I, 8000000);
	result = sample_line(idx);
	cpu_delay_us(J, 8000000); // Complete the reset sequence recovery
	return result; // Return sample presence pulse result
}

//-----------------------------------------------------------------------------
// Send a 1-Wire write bit. Provide 10us recovery time.
//
void OWWriteBit(unsigned char idx, int bit)
{
	if (bit)
	{
		// Write '1' bit
		drive_DQ_low(idx);
		cpu_delay_us(A, 8000000);
		release_the_bus(idx);
		cpu_delay_us(B, 8000000); // Complete the time slot and 10us recovery
	}
	else
	{
		// Write '0' bit
		drive_DQ_low(idx);
		cpu_delay_us(C, 8000000);
		release_the_bus(idx);
		cpu_delay_us(D, 8000000);
	}
}

//-----------------------------------------------------------------------------
// Read a bit from the 1-Wire bus and return it. Provide 10us recovery time.
//
int OWReadBit(unsigned char idx)
{
	int result;

	drive_DQ_low(idx);
	cpu_delay_us(A, 8000000);
	release_the_bus(idx);
	cpu_delay_us(E, 8000000);
	result = sample_line(idx);
	cpu_delay_us(F, 8000000); // Complete the time slot and 10us recovery

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