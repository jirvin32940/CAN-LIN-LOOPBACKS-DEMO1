/*****************************************************************************
 *
 * \file
 *
 * \brief PCA9952 driver for AVR32 UC3.
 *
 * This file is the PCA9952 driver.
 *
 * Copyright (c) 2009-2014 Atmel Corporation. All rights reserved.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * 4. This software may only be redistributed and used in connection with an
 *    Atmel microcontroller product.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \asf_license_stop
 *
 *****************************************************************************/
 /**
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
 */

//_____  I N C L U D E S ___________________________________________________
#include "board.h"
#include "compiler.h"
#include "gpio.h"
#include "conf_pca9952.h"
#include "pca9952.h"
#include "cycle_counter.h"
#if (UC3A3 || UC3A4 || UC3C || UC3L)
#include "twim.h"
#else
#include "twi.h"
#endif
/*! \brief Local driver data.
 */

static uint32_t cpu_hz;

/*! \brief Write device register content.
 *
 * \param reg_index Register address. Use macros as defined in the header file.
 * \param data Data that should be written to the device register.
 */
void PCA9952_write_reg(unsigned char topBotn, uint8_t reg_index, uint8_t data)
{
uint8_t pack[2];
twi_package_t twi_package;

	pack[0] = reg_index;
	pack[1] = data;

//7apr15	twi_package.chip = PCA9952_TWI_ADDRESS;

	if (topBotn == LED_TOP)
	{
		twi_package.chip = PCA9952_U7_TOPDRIVE_TWI_ADDRESS;
	}
	else if (topBotn == LED_BOTTOM)
	{
		twi_package.chip = PCA9952_U8_BOTDRIVE_TWI_ADDRESS;
	}

	twi_package.addr_length = 0;
	twi_package.buffer = &pack;
	twi_package.length = sizeof(pack);

	while(twi_master_write(PCA9952_TWI, &twi_package)!=TWI_SUCCESS);

	return;
}

/*! \brief Read device register content.
 *
 * \param reg_index Register address.
 * \returns Register content.
 */
uint8_t PCA9952_read_reg(unsigned char topBotn, uint8_t reg_index)
{
uint8_t data;
twi_package_t twi_package;

//7apr15	twi_package.chip = PCA9952_TWI_ADDRESS;

	if (topBotn == LED_TOP)
	{
		twi_package.chip = PCA9952_U7_TOPDRIVE_TWI_ADDRESS;
	}
	else if (topBotn == LED_BOTTOM)
	{
		twi_package.chip = PCA9952_U8_BOTDRIVE_TWI_ADDRESS;
	}

	twi_package.addr_length = 0;
	twi_package.buffer = &reg_index;
	twi_package.length = 1;
	while(twi_master_write(PCA9952_TWI, &twi_package)!=TWI_SUCCESS);
	/* We need a delay here to make this work although this is not
	* specified in the datasheet.
	* Also there seems to be a bug in the TWI module or the driver
	* since some delay here (code or real delay) adds about 500us
	* between the write and the next read cycle.
	*/
	cpu_delay_us(20, cpu_hz);

//7apr15 this was set above, no need to reassign	twi_package.chip = PCA9952_TWI_ADDRESS;
	twi_package.addr_length = 0;
	twi_package.buffer = &data;
	twi_package.length = 1;
	while(twi_master_read(PCA9952_TWI, &twi_package)!=TWI_SUCCESS);

	return data;
}


void PCA9952_init(void) //7apr15
{
	volatile uint8_t tmp1, tmp2, tmp3, tmp4;
	
	#define LED_DRIVER_CURRENT (0xC8/2)		//This setting puts us at 20mA on each channel. 
											//12apr15 cut the value in half because our power supply can't do full power for 4 shelves at the same time right now

	/* Store cpu frequency locally*/
//7apr15	cpu_hz = fcpu;

	//Note output is off at the chip level before coming into this routine, LED_OEn set high before calling this function from main()

	PCA9952_write_reg(LED_TOP, PCA9952_MODE1, 0);		//No autoincrement, normal mode not sleep, does not respond to sub or allcall addresses
	PCA9952_write_reg(LED_TOP, PCA9952_MODE2, 0);		//Dimming not blinking, change on stop not ack (don't really care but have to pick something)
	PCA9952_write_reg(LED_TOP, PCA9952_IREFALL, LED_DRIVER_CURRENT); //9apr15 drive at half current for now, power supply circuit needs modification
	PCA9952_write_reg(LED_TOP, PCA9952_LEDOUT0, 0);		//Setting all channels off for now
	PCA9952_write_reg(LED_TOP, PCA9952_LEDOUT1, 0);
	PCA9952_write_reg(LED_TOP, PCA9952_LEDOUT2, 0);
	PCA9952_write_reg(LED_TOP, PCA9952_LEDOUT3, 0);

	tmp1 = PCA9952_read_reg(LED_TOP, PCA9952_EFLAG0);	//TODO: just see what we get, will need to weave error checking into this system later
	tmp2 = PCA9952_read_reg(LED_TOP, PCA9952_EFLAG1);

	PCA9952_write_reg(LED_BOTTOM, PCA9952_MODE1, 0);		//No autoincrement, normal mode not sleep, does not respond to sub or allcall addresses
	PCA9952_write_reg(LED_BOTTOM, PCA9952_MODE2, 0);		//Dimming not blinking, change on stop not ack (don't really care but have to pick something)
	PCA9952_write_reg(LED_BOTTOM, PCA9952_IREFALL, LED_DRIVER_CURRENT); //9apr15 drive at half current for now, power supply circuit needs modification
	PCA9952_write_reg(LED_BOTTOM, PCA9952_LEDOUT0, 0);		//Setting all channels off for now
	PCA9952_write_reg(LED_BOTTOM, PCA9952_LEDOUT1, 0);
	PCA9952_write_reg(LED_BOTTOM, PCA9952_LEDOUT2, 0);
	PCA9952_write_reg(LED_BOTTOM, PCA9952_LEDOUT3, 0);

	tmp3 = PCA9952_read_reg(LED_BOTTOM, PCA9952_EFLAG0);	//TODO: just see what we get, will need to weave error checking into this system later
	tmp4 = PCA9952_read_reg(LED_BOTTOM, PCA9952_EFLAG1);

}


void PCA9952_channel(unsigned char topBotn, unsigned char channel, unsigned char onOffn);
void PCA9952_channel(unsigned char topBotn, unsigned char channel, unsigned char onOffn)
{
	unsigned char regIdx, regPos, regShadow, maskVal, writeVal;
	
	regIdx = (channel / 4); //LEDOUT0 controls channels 0..3, LEDOUT1 controls channels 4..7 etc.
	regPos = (channel % 4);
	
	switch(regIdx)
	{
		case 0:
			regShadow = PCA9952_read_reg(topBotn,PCA9952_LEDOUT0);
			break;
		case 1:
			regShadow = PCA9952_read_reg(topBotn,PCA9952_LEDOUT1);
			break;
		case 2:
			regShadow = PCA9952_read_reg(topBotn,PCA9952_LEDOUT2);
			break;
		case 3:
			regShadow = PCA9952_read_reg(topBotn,PCA9952_LEDOUT3);
			break;		
	}

	maskVal = 0xFF << (regPos * 2);
	maskVal = maskVal ^ 0xFF;
	regShadow &= maskVal;

	if (onOffn == LED_ON)
	{
		writeVal = (0x01 << (regPos * 2));
	}
	else if (onOffn == LED_OFF)
	{
		writeVal = 0;
	}
	
	regShadow |= writeVal;
	
	switch(regIdx)
	{
		case 0:
			PCA9952_write_reg(topBotn,PCA9952_LEDOUT0, regShadow);
			break;
		case 1:
			PCA9952_write_reg(topBotn,PCA9952_LEDOUT1, regShadow);
			break;
		case 2:
			PCA9952_write_reg(topBotn,PCA9952_LEDOUT2, regShadow);
			break;
		case 3:
			PCA9952_write_reg(topBotn,PCA9952_LEDOUT3, regShadow);
			break;
	}

}

void led_shelf(unsigned char shelf, unsigned char onOffn)
{
	switch(shelf)
	{
		case 0: //bottom of LED board 0 which is the upper board in the shelf, and top of LED board 1 which is the lower board in the shelf
			PCA9952_channel(LED_BOTTOM, 0, onOffn);
			PCA9952_channel(LED_BOTTOM, 1, onOffn);
			PCA9952_channel(LED_BOTTOM, 2, onOffn);
			PCA9952_channel(LED_TOP, 0, onOffn);
			PCA9952_channel(LED_TOP, 1, onOffn);
			break;

		case 1: //bottom of LED board 1 which is the upper board in the shelf, and top of LED board 2 which is the lower board in the shelf
			PCA9952_channel(LED_BOTTOM, 3, onOffn);
			PCA9952_channel(LED_BOTTOM, 4, onOffn);
			PCA9952_channel(LED_BOTTOM, 5, onOffn);
			PCA9952_channel(LED_TOP, 2, onOffn);
			PCA9952_channel(LED_TOP, 3, onOffn);
			break;

		case 2: //bottom of LED board 2 which is the upper board in the shelf, and top of LED board 3 which is the lower board in the shelf
			PCA9952_channel(LED_BOTTOM, 6, onOffn);
			PCA9952_channel(LED_BOTTOM, 7, onOffn);
			PCA9952_channel(LED_BOTTOM, 8, onOffn);
			PCA9952_channel(LED_TOP, 4, onOffn);
			PCA9952_channel(LED_TOP, 5, onOffn);
			break;

		case 3: //bottom of LED board 3 which is the upper board in the shelf, and top of LED board 4 which is the lower board in the shelf
			PCA9952_channel(LED_BOTTOM, 9, onOffn);
			PCA9952_channel(LED_BOTTOM, 10, onOffn);
			PCA9952_channel(LED_BOTTOM, 11, onOffn);
			PCA9952_channel(LED_TOP, 6, onOffn);
			PCA9952_channel(LED_TOP, 7, onOffn);
			break;
	}	
	
}

