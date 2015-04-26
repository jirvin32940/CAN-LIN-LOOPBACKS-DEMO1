/**
 * \file
 *
 * \brief AVR UC3C CAN-LIN Loopback Demo
 *
 * Copyright (c) 2011-2014 Atmel Corporation. All rights reserved.
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
 */
/*! \mainpage
 * \section intro Introduction
 * This is the documentation for CAN and LIN loopback demo application running
 * on the UC3C_EK development kit.
 * - Step 1: The potentiometer value is read and displayed on the LCD screen.
 * - Step 2: The value of the potentiometer is sent through CAN network.
 * - Step 3: The content of the received message on CAN network is displayed on
 *   the LCD screen and corresponds to the screen observed in step 1.
 * - Step 4: The same message is also transmitted on LIN bus
 * - Step 5: The content of the message received on LIN network is displayed on
 *   the LCD screen and should correspond to the screen observed on step 1,3.
 * \section files Main Files
 * - main.c: Main File;
 * - can_task.c: CAN Task Management;
 * - lin_task.c: LIN Task Management;
 * - controller.c: Qtouch Controller Management;
 * - gui.c: GUI Display Management;
 * - <A href="http://www.atmel.com/dyn/resources/prod_documents/doc32137.pdf"
 *   style="text-decoration:none"><b>doc32137.pdf</b></A>: AVR32907 UC3C-EK
 *   Getting Started file - how to setup and run this application;
 *
 * \section compilinfo Compilation Information
 * This software is written for GNU GCC for AVR32 and for IAR Embedded Workbench
 * for Atmel AVR32. Other compilers may or may not work.
 *
 * \section deviceinfo Device Information
 * All AVR32 AT32UC3C devices can be used.
 *
 * \section configinfo Configuration Information
 * This example has been tested with the following configuration:
 * - UC3C_EK evaluation kit;
 * - CPU clock: 16 MHz;
 * - USART2 (on UC3C_EK) abstracted with a USB CDC connection to a PC;
 * - PC terminal settings:
 *   - 57600 bps,
 *   - 8 data bits,
 *   - no parity bit,
 *   - 1 stop bit,
 *   - no flow control.
 *
 * \section contactinfo Contact Information
 * For further information, visit
 * <A href="http://www.atmel.com/avr">Atmel AVR</A>.\n
 */
 /**
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
 */

#include "compiler.h"
// 6apr15 #include "board.h"
#include "power_clocks_lib.h"
//6 apr15 #include "dsp.h"
//6 apr15 #include "gui.h"
//6 apr15 #include "controller.h"
#include "gpio.h"
#include "ss_print_funcs.h" //8apr15 changed from print_funcs.h
#include "flashc.h"
#include "adcifa.h"
#include "twim.h"
// 6apr15 #include "conf_at42qt1060.h"
#include "conf_pca9952.h" //6apr15
#include "pca9952.h" //7apr15
//6 apr15 #include "lin_task.h"
//6 apr15 #include "can_task.h"
//8apr15 #include "conf_demo.h"
#include "conf_sealshield.h"	//8apr15
#include "cycle_counter.h"		//8apr15	
#include "usart.h"				//9apr15
#include "serial_id_ds2411.h"	//9apr15

//6apr15 A_ALIGNED dsp16_t signal1_buf[BUFFER_LENGTH];
//6apr15 A_ALIGNED dsp16_t signal4_buf[BUFFER_LENGTH];

int16_t bluesense0_buf[BUFFER_LENGTH]; //6apr15 remember we changed dsp16_t to int16_t without all the aligned stuff in case this does something squirrely
int16_t bluesense1_buf[BUFFER_LENGTH]; //6apr15
int16_t bluesense2_buf[BUFFER_LENGTH]; //6apr15
int16_t bluesense3_buf[BUFFER_LENGTH]; //6apr15


unsigned char CMD_READY[7] =	{0x55, 0xAA, 0x91, 0x00, 0x00, 0x00, 0x00};
unsigned char CMD_CLEAN[7] =	{0x55, 0xAA, 0x92, 0x00, 0x00, 0x00, 0x00};
unsigned char CMD_CLEANING[7] = {0x55, 0xAA, 0x93, 0x00, 0x00, 0x00, 0x00};
unsigned char CMD_DIRTY[7] =	{0x55, 0xAA, 0x94, 0x00, 0x00, 0x00, 0x00};
unsigned char CMD_ERROR[7] =	{0x55, 0xAA, 0x95, 0x00, 0x00, 0x00, 0x00};
unsigned char CMD_SHELF1[7] =	{0x55, 0xAA, 0x96, 0x00, 0x00, 0x00, 0x00};
unsigned char CMD_SHELF2[7] =	{0x55, 0xAA, 0x97, 0x00, 0x00, 0x00, 0x00};
unsigned char CMD_SHELF3[7] =	{0x55, 0xAA, 0x98, 0x00, 0x00, 0x00, 0x00};
unsigned char CMD_SHELF4[7] =	{0x55, 0xAA, 0x99, 0x00, 0x00, 0x00, 0x00};
unsigned char CMD_CLEAR[7] =	{0x55, 0xAA, 0xCE, 0x00, 0x00, 0x00, 0x00}; //experiment 11apr15
	
unsigned char* cmdPtrArray[10] = {
	&CMD_READY[0],
	&CMD_CLEAN[0],
	&CMD_CLEANING[0],
	&CMD_DIRTY[0],
	&CMD_ERROR[0],
	&CMD_SHELF1[0],
	&CMD_SHELF2[0],
	&CMD_SHELF3[0],
	&CMD_SHELF4[0],
	&CMD_CLEAR
};

enum {
	IDX_READY,
	IDX_CLEAN,
	IDX_CLEANING,
	IDX_DIRTY,
	IDX_ERROR,
	IDX_SHELF1,
	IDX_SHELF2,
	IDX_SHELF3,
	IDX_SHELF4,
	IDX_CLEAR
	
};


volatile U16 adc_current_conversion;

#define SEALSHIELD_DOOR_LATCH	AVR32_PIN_PB30
#define SEALSHIELD_ACTION_PB	AVR32_PIN_PB31
#define SEALSHIELD_DEBUG_LED	AVR32_PIN_PD28
#define SEALSHIELD_PSUPPLY_ONn	AVR32_PIN_PA23
#define SEALSHIELD_LED_OEn		AVR32_PIN_PA22
#define SEALSHIELD_MFP			AVR32_PIN_PA21	//set to 1 for 1X, set to 0 for 4X

#define SS_DOOR_LATCHED (!gpio_get_pin_value(SEALSHIELD_DOOR_LATCH)) //12apr15 this is the correct sense for the equipment going to the show
#define SS_ACTION_PB	(!gpio_get_pin_value(SEALSHIELD_ACTION_PB)) //12apr15 this is the correct sense for the equipment going to the show

#define NUM_LED_BOARDS			5
#define NUM_LED_BOARD_SIDES		8
#define NUM_SHELVES				4	//Shelf 0 is board 0 bottom + board 1 top
									//Shelf 1 is board 1 bottom + board 2 top
									//Shelf 2 is board 2 bottom + board 3 top
									//Shelf 3 is board 3 bottom + board 4 top
									//Board 0 top and board 4 bottom are not used

enum {
	SHELF_ACTIVE,
	SHELF_INACTIVE
};

unsigned char brdSideWithinLifetimeLimit[NUM_LED_BOARD_SIDES];
unsigned char devicesPresentOnShelf[NUM_SHELVES];
unsigned char shelfActive[NUM_SHELVES];


void display_text(unsigned char idx)
{
	for (int i = 0; i<7; i++)
	{
		usart_putchar(DISPLAY_USART, ((unsigned char) ((*(cmdPtrArray[idx]+i)))));
	}
	
}

void init_io(void)
{
	uint32_t ioFlags;
	
	
	ioFlags = (GPIO_DIR_INPUT);
	gpio_configure_pin(SEALSHIELD_DOOR_LATCH, ioFlags);

	ioFlags = (GPIO_DIR_INPUT);
	gpio_configure_pin(SEALSHIELD_ACTION_PB, ioFlags);

	ioFlags = (GPIO_DIR_OUTPUT | GPIO_INIT_HIGH);
	gpio_configure_pin(SEALSHIELD_SERIAL_ID0, ioFlags);

	ioFlags = (GPIO_DIR_OUTPUT | GPIO_INIT_HIGH);
	gpio_configure_pin(SEALSHIELD_SERIAL_ID1, ioFlags);

	ioFlags = (GPIO_DIR_OUTPUT | GPIO_INIT_HIGH);
	gpio_configure_pin(SEALSHIELD_SERIAL_ID2, ioFlags);

	ioFlags = (GPIO_DIR_OUTPUT | GPIO_INIT_HIGH);
	gpio_configure_pin(SEALSHIELD_SERIAL_ID3, ioFlags);

	ioFlags = (GPIO_DIR_OUTPUT | GPIO_INIT_HIGH);
	gpio_configure_pin(SEALSHIELD_SERIAL_ID4, ioFlags);

	ioFlags = (GPIO_DIR_OUTPUT | GPIO_INIT_LOW);
	gpio_configure_pin(SEALSHIELD_DEBUG_LED, ioFlags);

	ioFlags = (GPIO_DIR_OUTPUT | GPIO_INIT_HIGH);
	gpio_configure_pin(SEALSHIELD_PSUPPLY_ONn, ioFlags);

	ioFlags = (GPIO_DIR_OUTPUT | GPIO_INIT_HIGH);
	gpio_configure_pin(SEALSHIELD_LED_OEn, ioFlags);

	ioFlags = (GPIO_DIR_OUTPUT | GPIO_INIT_LOW); //high=1x multiplier, low=4x multiplier 10apr15
	gpio_configure_pin(SEALSHIELD_MFP, ioFlags);

}


unsigned char ledBrdSerialID[NUM_LED_BOARDS][8]; //5 boards, 8 bit family code + 48 bit serial number each board + 8 bit CRC

/* LED board side designations: 2 sides per shelf */
enum {
	LED_BRD_0_BOT,	//shelf 0
	LED_BRD_1_TOP,	//shelf 0
	LED_BRD_1_BOT,	//shelf 1
	LED_BRD_2_TOP,	//shelf 1
	LED_BRD_2_BOT,	//shelf 2
	LED_BRD_3_TOP,	//shelf 2
	LED_BRD_3_BOT,	//shelf 3
	LED_BRD_4_TOP	//shelf 3
};

int ledBoardPresent[NUM_LED_BOARDS];

/* One serial ID chip per board */
void read_serial_ids(void)
{
	/*
	 * Check for LED board presence by issuing a reset to the serial ID chip and checking for a response.
	 */
	
	SetSpeed(1); //1==standard speed, not overdrive
	
	ledBoardPresent[0] = !OWTouchReset(0);
	ledBoardPresent[1] = !OWTouchReset(1);
	ledBoardPresent[2] = !OWTouchReset(2);
	ledBoardPresent[3] = !OWTouchReset(3);
	ledBoardPresent[4] = !OWTouchReset(4);
	
	for (int i=0; i<NUM_LED_BOARDS; i++)
	{
		if (ledBoardPresent[i])
		{
			OWWriteByte(i, 0x33); //Read ID command
			
			for (int j=0; j<8; j++)
			{
				ledBrdSerialID[i][j] = OWReadByte(i);
			}
		}
	}
}

/* LEDs we are using are rated for up to 2000 hours */
//TODO: Note part number here
enum {
	LED_BOARD_SIDE_PAST_LIFETIME_LIMIT,
	LED_BOARD_SIDE_WITHIN_LIFETIME_LIMIT
};

/* Each side of an LED board will get different usage */
unsigned char check_led_brd_side_lifetime(unsigned char ledBrdPosition)
{
	/*
	 * TODO: Find the record for this board's serial ID number, and check the usage hours and see if we
	 *			are past the 2000 hour mark. If we are, this board is considered un-usuable until it is
	 *			refurbished. This will require variable array storage. I could also create a linked
	 *			list to go with it to sort the IDs in numerial order to make them easier to find, otherwise
	 *			have to sort through the whole list every time, could get slow.
	 */
	
	return LED_BOARD_SIDE_WITHIN_LIFETIME_LIMIT;	//TODO: Every board is within limit for now. Perhaps hardcode one as out of spec
													//		for demo purposes.
 	
}

/* Aggregate the information */
void check_led_brd_side_lifetimes(void)
{
	for (int i=0; i<NUM_LED_BOARD_SIDES; i++)
	{
		brdSideWithinLifetimeLimit[i] = check_led_brd_side_lifetime(i);
	}
}



// ADC Configuration
adcifa_opt_t adc_config_t = {
	.frequency                = 1000000,        // ADC frequency (Hz)
	.reference_source         = ADCIFA_ADCREF0, // Reference Source
	.sample_and_hold_disable  = false,    		// Disable Sample and Hold Time
	.single_sequencer_mode    = false,   		// Single Sequencer Mode
	.free_running_mode_enable = false,    		// Free Running Mode
	.sleep_mode_enable        = false,    		// Sleep Mode
	.mux_settle_more_time     = false     		// Multiplexer Settle Time
};

// Sequencer Configuration: same for all sequencers
adcifa_sequencer_opt_t adcifa_sequence_opt = {
	NUMBER_OF_INPUTS_ADC_SEQ0,
	ADCIFA_SRES_12B,
	ADCIFA_TRGSEL_SOFT,
	ADCIFA_SOCB_ALLSEQ,
	ADCIFA_SH_MODE_OVERSAMP,
	ADCIFA_HWLA_NOADJ,
	ADCIFA_SA_NO_EOS_SOFTACK
};

int16_t adc_values_seq0[NUMBER_OF_INPUTS_ADC_SEQ0];
adcifa_sequencer_conversion_opt_t
adcifa_sequence_conversion_opt_seq0_shelf1[NUMBER_OF_INPUTS_ADC_SEQ0] = {
	{
		INPUT1_ADC_INP,
		INPUT1_ADC_INN,
		ADCIFA_SHG_8
	}
};

int16_t adc_values_seq1[NUMBER_OF_INPUTS_ADC_SEQ1];
adcifa_sequencer_conversion_opt_t
adcifa_sequence_conversion_opt_seq1_shelf2[NUMBER_OF_INPUTS_ADC_SEQ1] = {
	{
		INPUT2_ADC_INP,
		INPUT2_ADC_INN,
		ADCIFA_SHG_8
	}
};

adcifa_sequencer_conversion_opt_t
adcifa_sequence_conversion_opt_seq0_shelf3[NUMBER_OF_INPUTS_ADC_SEQ0] = {
	{
		INPUT3_ADC_INP,
		INPUT3_ADC_INN,
		ADCIFA_SHG_8
	}
};

adcifa_sequencer_conversion_opt_t
adcifa_sequence_conversion_opt_seq1_shelf4[NUMBER_OF_INPUTS_ADC_SEQ1] = {
	{
		INPUT4_ADC_INP,
		INPUT4_ADC_INN,
		ADCIFA_SHG_8 //TODO: what is this supposed to be?? This is gain conversion, figure out what it needs to be set to and set all bluesense channels the same way
	}
};

volatile avr32_adcifa_t *adcifa = &AVR32_ADCIFA; // ADCIFA IP registers address

int16_t adc_process_task(unsigned char shelfIdx)
{
	int32_t i;

	volatile int16_t retVal;

	switch (shelfIdx)
	{
		case 0:
			// Configure ADCIFA sequencer 0 shelf 1
			adcifa_configure_sequencer(adcifa, 0,
			&adcifa_sequence_opt,
			adcifa_sequence_conversion_opt_seq0_shelf1);

			// Start ADCIFA sequencer 0
			adcifa_start_sequencer(adcifa, 0);

			// Get Values from sequencer 0
			if (adcifa_get_values_from_sequencer(adcifa,
			0,
			&adcifa_sequence_opt,
			adc_values_seq0) == ADCIFA_STATUS_COMPLETED) {
				for (i=BUFFER_LENGTH-1;i>=1;i--) {
					bluesense0_buf[i] = bluesense0_buf[i-1];
				}
				bluesense0_buf[0] = adc_values_seq0[0];
				retVal = bluesense0_buf[0];
			}
			break;
		
		case 1:
			// Configure ADCIFA sequencer 1 shelf 2
			adcifa_configure_sequencer(adcifa, 1,
			&adcifa_sequence_opt,
			adcifa_sequence_conversion_opt_seq1_shelf2);

			// Start ADCIFA sequencer 1
			adcifa_start_sequencer(adcifa, 1);

			// Get Values from sequencer 1
			if (adcifa_get_values_from_sequencer(adcifa,
			1,
			&adcifa_sequence_opt,
			adc_values_seq1) == ADCIFA_STATUS_COMPLETED) {
				for (i=BUFFER_LENGTH-1;i>=1;i--) {
					bluesense1_buf[i] = bluesense1_buf[i-1];
				}
				bluesense1_buf[0] = adc_values_seq1[0];
			}
			retVal = bluesense1_buf[0];
			break;
		
		case 2:
			// Configure ADCIFA sequencer 0 shelf 3
			adcifa_configure_sequencer(adcifa, 0,
			&adcifa_sequence_opt,
			adcifa_sequence_conversion_opt_seq0_shelf3);

			// Start ADCIFA sequencer 0
			adcifa_start_sequencer(adcifa, 0);

			// Get Values from sequencer 2
			if (adcifa_get_values_from_sequencer(adcifa,
			0,
			&adcifa_sequence_opt,
			adc_values_seq0) == ADCIFA_STATUS_COMPLETED) {
				for (i=BUFFER_LENGTH-1;i>=1;i--) {
					bluesense2_buf[i] = bluesense2_buf[i-1];
				}
				bluesense2_buf[0] = adc_values_seq0[0];
			}
			retVal = bluesense2_buf[0];
			break;
		
		case 3:
			// Configure ADCIFA sequencer 1 shelf 4
			adcifa_configure_sequencer(adcifa, 1,
			&adcifa_sequence_opt,
			adcifa_sequence_conversion_opt_seq1_shelf4);

			// Start ADCIFA sequencer 1
			adcifa_start_sequencer(adcifa, 1);

			// Get Values from sequencer 1
			if (adcifa_get_values_from_sequencer(adcifa,
			1,
			&adcifa_sequence_opt,
			adc_values_seq1) == ADCIFA_STATUS_COMPLETED) {
				for (i=BUFFER_LENGTH-1;i>=1;i--) {
					bluesense3_buf[i] = bluesense3_buf[i-1];
				}
				bluesense3_buf[0] = adc_values_seq1[0];
			}
			retVal = bluesense3_buf[0];
			break;
	}
	
	return retVal;

}




enum {
	NO_DEVICES_PRESENT,
	DEVICES_PRESENT
};

unsigned char check_shelf_for_devices(unsigned char shelfPosition)
{
	U16 bluesense;
	unsigned char retVal;
	
	led_shelf(shelfPosition, LED_ON); //TODO: do we finish this task fast enough to not check the door latch in here? Can't have LEDs on if the door opens
	
	cpu_delay_ms(50, 8000000);
		
	//Read bluesense for this shelf
	bluesense = 0;
	bluesense = adc_process_task(shelfPosition);

	led_shelf(shelfPosition, LED_OFF);
	
	retVal = NO_DEVICES_PRESENT;

	switch(shelfPosition)
	{
		case 0:
			if (bluesense > 0x800) {
				retVal = DEVICES_PRESENT;
			}
			break;
		case 1:
			if (bluesense > 0x800) {
				retVal = DEVICES_PRESENT;
			}
			break;
		case 2:
			if (bluesense > 0x800) {
				retVal = DEVICES_PRESENT;
			}
			break;
		case 3:
			if (bluesense > 0x800) {
				retVal = DEVICES_PRESENT;
			}
			break;
	}
	

	return retVal;
}

void check_shelves_for_devices(void)
{
	for (int i=0; i<NUM_SHELVES; i++)
	{
		devicesPresentOnShelf[i] = check_shelf_for_devices(i);
	}
}

unsigned char numActiveShelves;

unsigned char shelfPresent[NUM_SHELVES] = {0,0,0,0};


unsigned char topEflag0, topEflag1, botEflag0, botEflag1;

void set_shelves_active_inactive(void)
{
	numActiveShelves = 0;
	
	unsigned char tmp1, tmp2, tmp3, tmp4;
	
/*
 * Test which shelves are present in the system.
 * NOTE: We should do this at the board level, being sloppy
 * before the trade show, just trying to get it done. 10apr15
 *
 * Actually going to abandon this as a strategy for detecting whether
 * an LED board is present, problematic. Go back to trying to detect
 * LED board presence with the ID chip.
 */

#if 0	//TODO: NOTE we can't do this right now, turn all the shelves on full power at once, because the power supply can't supply it. Also not sure how to use the LED driver chip to check for LED shorts and opens
		//it's more complicated than it looks, fix this later.
	led_shelf(0, LED_ON);
	led_shelf(1, LED_ON);
	led_shelf(2, LED_ON);
	led_shelf(3, LED_ON);
	
	PCA9952_write_reg(LED_TOP, PCA9952_MODE2, 0x40); //starts fault test
	PCA9952_write_reg(LED_BOTTOM, PCA9952_MODE2, 0x40); //starts fault test
	
	while (1)
	{
		tmp1 = PCA9952_read_reg(LED_TOP, PCA9952_MODE2);
		
		if ((tmp1 & 0x40) == 0)
		{
			topEflag0 = PCA9952_read_reg(LED_TOP, PCA9952_EFLAG0);
			topEflag1 = PCA9952_read_reg(LED_TOP, PCA9952_EFLAG1);
			
			break; //fault test for LED_TOP board is complete
		}
		
	}
	
	while (1)
	{
		tmp3 = PCA9952_read_reg(LED_BOTTOM, PCA9952_MODE2);
		
		if ((tmp3 & 0x40) == 0)
		{
			botEflag0 = PCA9952_read_reg(LED_BOTTOM, PCA9952_EFLAG0);
			botEflag1 = PCA9952_read_reg(LED_BOTTOM, PCA9952_EFLAG1);
			
			break; //fault test for LED_TOP board is complete
		}
		
	}
	
	led_shelf(0, LED_OFF);
	led_shelf(1, LED_OFF);
	led_shelf(2, LED_OFF);
	led_shelf(3, LED_OFF);
	
#endif	
	
	/*
	 * The rest of the evaluation
	 */
	
	
	for (int i=0; i<NUM_SHELVES; i++)
	{
		shelfActive[i] = SHELF_INACTIVE;
		
	}
	
	/* check shelf 0 */
	if (ledBoardPresent[0] &&
		ledBoardPresent[1] &&
		devicesPresentOnShelf[0] && 
		brdSideWithinLifetimeLimit[LED_BRD_0_BOT] &&
		brdSideWithinLifetimeLimit[LED_BRD_1_TOP] )
	{
		shelfActive[0] = SHELF_ACTIVE;
		numActiveShelves++;
		print_ssdbg("Shelf 0 active\r\n");
	}
	
	/* check shelf 1 */
	
	if (ledBoardPresent[1] &&
		ledBoardPresent[2] &&
	devicesPresentOnShelf[1] &&
	brdSideWithinLifetimeLimit[LED_BRD_1_BOT] &&
	brdSideWithinLifetimeLimit[LED_BRD_2_TOP] )
	{
		shelfActive[1] = SHELF_ACTIVE;
		numActiveShelves++;
		print_ssdbg("Shelf 1 active\r\n");
	}
	
	/* check shelf 2 */
	
	if (ledBoardPresent[2] &&
		ledBoardPresent[3] &&
	devicesPresentOnShelf[2] &&
	brdSideWithinLifetimeLimit[LED_BRD_2_BOT] &&
	brdSideWithinLifetimeLimit[LED_BRD_3_TOP] )
	{
		shelfActive[2] = SHELF_ACTIVE;
		numActiveShelves++;
		print_ssdbg("Shelf 2 active\r\n");
	}
	
	/* check shelf 3 */
	
	if (ledBoardPresent[3] &&
		ledBoardPresent[4] &&
	devicesPresentOnShelf[3] &&
	brdSideWithinLifetimeLimit[LED_BRD_3_BOT] &&
	brdSideWithinLifetimeLimit[LED_BRD_4_TOP] )
	{
		shelfActive[3] = SHELF_ACTIVE;
		numActiveShelves++;
		print_ssdbg("Shelf 3 active\r\n");
	}
}

unsigned char num_active_shelves(void)
{
	return numActiveShelves;
}


/*! \brief Initializes the MCU system clocks.
 */
#if 0 //7apr15 we are using RC8M, not an external crystal
void init_sys_clocks(void)
{
	pcl_switch_to_osc(PCL_OSC0, FOSC0, OSC0_STARTUP);
}
#endif //7apr15


/*
 * Using RC8M (internal 8MHz)
 */
void init_sys_clocks(void)
{
	scif_gclk_opt_t gclkOpt = {SCIF_GCCTRL_RC8M, 0,0};
	
	// Start the 8Mhz Oscillator
	scif_start_rc8M();
	// Set the main clock source as being RC8MHz.
	pm_set_mclk_source(PM_CLK_SRC_RC8M);	


	/* put the clock out on PC19 so we can check to make sure we set it up correctly */
	//Note this code comes from ASF example AVR32 SCIF example 3
	scif_start_gclk(AVR32_SCIF_GCLK_GCLK0PIN, &gclkOpt);
	gpio_enable_module_pin(AVR32_SCIF_GCLK_0_1_PIN, AVR32_SCIF_GCLK_0_1_FUNCTION);
	
}

volatile bool input_fft_view = false;
volatile bool output_fft_view = false;
volatile bool zoom_view = false;
volatile int32_t zoom_view_id;

enum state_master {
	STATE_IDLE,
	STATE_SOURCE1,
	STATE_OUTPUT1,
	STATE_OUTPUT2,
	STATE_OUTPUT3
};

enum state_function {
	STATE_FCT_IDLE,
	STATE_FCT_FUNCTION1,
	STATE_FCT_FUNCTION2,
	STATE_FCT_FUNCTION3,
	STATE_FCT_FUNCTION4,
	STATE_FCT_ZOOM
};

static enum state_master state = STATE_IDLE;
static enum state_function state_fct = STATE_FCT_IDLE;
static bool new_state_fct = false;

/*! \brief Global State Machine:
 *        - Function Idle
 *        - Function Zoom
 */

static bool state_machine_global(int source_id, enum state_function *state)
{
#if 0 //6apr15
	switch (*state) {
		case STATE_FCT_IDLE:
			if (source_id == GUI_SOURCE1_ID) {
				if (new_state_fct) {
					gui_set_selection(GUI_SOURCE1_ID);
				}
			}
			else if (source_id == GUI_OUTPUT1_ID) {
				if (new_state_fct) {
					gui_set_selection(GUI_OUTPUT1_ID);
				}
			}
			else if (source_id == GUI_OUTPUT2_ID) {
				if (new_state_fct) {
					gui_set_selection(GUI_OUTPUT2_ID);
				}
			}
			else if (source_id == GUI_OUTPUT3_ID) {
				if (new_state_fct) {
					gui_set_selection(GUI_OUTPUT3_ID);
				}
			}
			break;
		// Not Implemented
		case STATE_FCT_FUNCTION1:
			break;
		// Not Implemented
		case STATE_FCT_FUNCTION2:
			break;
		// Not Implemented
		case STATE_FCT_FUNCTION3:
			break;
		// Not Implemented
		case STATE_FCT_FUNCTION4:
			break;
		// Zoom
		case STATE_FCT_ZOOM:
			if (new_state_fct) {
				zoom_view = true;
				if (source_id == GUI_SOURCE1_ID)
					zoom_view_id = GUI_SOURCE1_ID;
				else if (source_id == GUI_OUTPUT1_ID)
					zoom_view_id = GUI_OUTPUT1_ID;
				else if (source_id == GUI_OUTPUT2_ID)
					zoom_view_id = GUI_OUTPUT2_ID;
				else if (source_id == GUI_OUTPUT3_ID)
					zoom_view_id = GUI_OUTPUT3_ID;
			}
			break;
	}
#endif //6apr15
	return true;
}

/*! \brief Navigation State Machine:
 *        - STATE_SOURCE1, STATE_OUTPUT1, STATE_OUTPUT2, OUTPUT3
 *
 */
static void state_machine_task(void)
{
#if 0 //6apr15
	// Set function state
	if (controller_key_fct5()) {
		state_fct = STATE_FCT_ZOOM;
		new_state_fct = true;
	}
	else if (controller_key_fct1()) {
		state_fct = STATE_FCT_IDLE;
		state = STATE_SOURCE1;
		new_state_fct = true;
	}
	else if (controller_key_fct2()) {
		state_fct = STATE_FCT_IDLE;
		state = STATE_OUTPUT1;
		new_state_fct = true;
	}
	else if (controller_key_fct3()) {
		state_fct = STATE_FCT_IDLE;
		state = STATE_OUTPUT2;
		new_state_fct = true;
	}
	else if (controller_key_fct4()) {
		state_fct = STATE_FCT_IDLE;
		state = STATE_OUTPUT3;
		new_state_fct = true;
	}
	// Clear Zoom state if on and a key is pressed
	if (zoom_view && !controller_key_fct5()) {
		zoom_view = false;
		gui_clear_view();
		new_state_fct = true;
		state_fct = STATE_FCT_IDLE;
	}

	switch (state) {
		case STATE_IDLE:
			break;
		case STATE_SOURCE1:
			if (!state_machine_global(GUI_SOURCE1_ID, &state_fct))
				return;
			break;
		case STATE_OUTPUT1:
			if (!state_machine_global(GUI_OUTPUT1_ID, &state_fct))
				return;
			break;
		case STATE_OUTPUT2:
			if (!state_machine_global(GUI_OUTPUT2_ID, &state_fct))
				return;
			break;
		case STATE_OUTPUT3:
			if (!state_machine_global(GUI_OUTPUT3_ID, &state_fct))
				return;
			break;
	}
	new_state_fct = false;
#endif //6apr15
}

/*! \brief ADC Process Init
 *
 *
 */
void adc_process_init(void)
{
	// GPIO pin/adc-function map.
	static const gpio_map_t ADCIFA_GPIO_MAP = {
		{AVR32_ADCREF0_PIN,AVR32_ADCREF0_FUNCTION},
		{AVR32_ADCREFP_PIN,AVR32_ADCREFP_FUNCTION},
		{AVR32_ADCREFN_PIN,AVR32_ADCREFN_FUNCTION},
		{INPUT1_ADC_PIN, INPUT1_ADC_FUNCTION},
		{INPUT2_ADC_PIN, INPUT2_ADC_FUNCTION},
		{INPUT3_ADC_PIN, INPUT3_ADC_FUNCTION},	//8apr15 TODO is this right??
		{INPUT4_ADC_PIN, INPUT4_ADC_FUNCTION}	//8apr15 TODO is this right??
	};

	// Assign and enable GPIO pins to the ADC function.
	gpio_enable_module(ADCIFA_GPIO_MAP,
			sizeof(ADCIFA_GPIO_MAP) / sizeof(ADCIFA_GPIO_MAP[0]));

	// Get ADCIFA Factory Configuration
	adcifa_get_calibration_data(adcifa, &adc_config_t);

	// Configure ADCIFA core
//debug 10apr15 	adcifa_configure(adcifa, &adc_config_t, FOSC0);
	adcifa_configure(adcifa, &adc_config_t, 8000000); //10apr15

}

/*! \brief ADC Process Task
 *
 *
 */
/*! \brief TWI Initialization for QTouch Controller
 *
 *
 */
static void twi_init(void)
{
	const gpio_map_t PCA9952_TWI_GPIO_MAP = {
		{PCA9952_TWI_SCL_PIN, PCA9952_TWI_SCL_FUNCTION},
		{PCA9952_TWI_SDA_PIN, PCA9952_TWI_SDA_FUNCTION}
	};

//7apr15	const twi_options_t PCA9952_TWI_OPTIONS = {
	twi_options_t PCA9952_TWI_OPTIONS = { //7apr15 make this *not* a const so we can change it and rerun twi_master_init() if necessary
		.pba_hz = FPBA_HZ,
		.speed = PCA9952_TWI_MASTER_SPEED,
//7apr15		.chip = PCA9952_TWI_ADDRESS,
		.chip = PCA9952_U7_TOPDRIVE_TWI_ADDRESS, //7apr15
		.smbus        = false,
	};

	// Assign I/Os to TWI.
	gpio_enable_module(PCA9952_TWI_GPIO_MAP,
	sizeof(PCA9952_TWI_GPIO_MAP) / sizeof(PCA9952_TWI_GPIO_MAP[0]));
	// Initialize as master.
	twi_master_init(PCA9952_TWI, &PCA9952_TWI_OPTIONS);
	
	
	PCA9952_TWI_OPTIONS.chip = PCA9952_U8_BOTDRIVE_TWI_ADDRESS;
	// Initialize as master.
	twi_master_init(PCA9952_TWI, &PCA9952_TWI_OPTIONS);
	
	
}


enum {
	STATE_SS_IDLE,
	STATE_DOOR_LATCHED,
	STATE_ACTION_PB_PRESSED,
	STATE_ACTION_PB_RELEASED,
	STATE_START_SANITIZE,
	STATE_SANITIZE_1,
	STATE_SANITIZE_2,
	STATE_START_CLEAN,
	STATE_CLEAN,
	STATE_SHUTDOWN_PROCESSES
};

unsigned long calc_sanitize_time(unsigned char shelfIdx)
{
	uint32_t cyclesPerSec;
	
	cyclesPerSec = cpu_ms_2_cy(1000, 8000000);
	
	//TODO: put the real calculation here based on hours of usage
	
	return (8*cyclesPerSec); //fudge for now, show the string and let it wrap once, looks better for the show
	
}

t_cpu_time timerShelf0, timerShelf1, timerShelf2, timerShelf3, timerClean, timerDebugLed;

unsigned char shelfActive[NUM_SHELVES];
unsigned long shelfTimerInitSeconds[NUM_SHELVES];
t_cpu_time shelfTimer[NUM_SHELVES];
t_cpu_time* shelfTimerPtr[NUM_SHELVES] = {&shelfTimer[0], &shelfTimer[1], &shelfTimer[2], &shelfTimer[3]};

unsigned char sealShieldState;
unsigned char anyShelvesStillSanitizing;


void door_latch_open_kill_all_shelves(void)
{
	led_shelf(0, LED_OFF);
	led_shelf(1, LED_OFF);
	led_shelf(2, LED_OFF);
	led_shelf(3, LED_OFF);
}

unsigned char firstTimeThrough = 1;

/*! \brief Main File Section:
 *          - Initialization (CPU, TWI, Usart,...)
 *          - Main loop with task management (CAN, LIN, ADC)
 */
int main(void)
{
	static unsigned char sanitizeIdx = 0;
	
	// Initialize System Clock
	init_sys_clocks();

	init_io();
	

	// Initialize USART
	init_ssdbg_rs232(FPBA_HZ);
	init_display_rs232(FPBA_HZ);

	// Print Startup Message
	print_ssdbg("SEAL SHIELD DEMO \r\n Copyright (c) 2015 Technical Solutions Group, Inc.\r\n");
	display_text(IDX_READY);
	
	// Initialize ADC for bluesense channels which are used to see if there are any devices (phones, tablets, etc.) on the shelves
	adc_process_init();

	
	// Initialize Interrupts
	irq_initialize_vectors(); //TODO: probably remove 5apr15

	cpu_irq_enable();

	// Initialize TWI Interface
	twi_init();

	gpio_set_pin_high(SEALSHIELD_LED_OEn); //make sure outputs are disabled at the chip level
	PCA9952_init();

	
#if 0 //TODO: probably remove 5apr15
	gui_init(FCPU_HZ, FHSB_HZ, FPBB_HZ, FPBA_HZ); // GUI, Controller and DSP process init
	controller_init(FCPU_HZ, FHSB_HZ, FPBA_HZ, FPBB_HZ);
	lin_task_init(); // Initialize LIN Interface
	can_task_init(); // Initialize CAN Interface
#endif

	sealShieldState = STATE_SS_IDLE;
	
	//using this structure makes the timer IDs index-able
	shelfTimerPtr[0] = &timerShelf0;
	shelfTimerPtr[1] = &timerShelf1;
	shelfTimerPtr[2] = &timerShelf2;
	shelfTimerPtr[3] = &timerShelf3;
	
	gpio_set_pin_low(SEALSHIELD_LED_OEn); //...and we are live!
	gpio_set_pin_low(SEALSHIELD_PSUPPLY_ONn); //turn the leds on first and then the power supply
	
	cpu_set_timeout(SS_ONE_SECOND/2, &timerDebugLed);


	// Main loop
	while (true) {

#if 0 //TODO: probably remove 5apr15
		lin_task(); // Call Lin Task for communication management
		can_task(); // Call CAN Task for communication management
		gui_task(); // Call Gui Task for display update
		controller_task(); // Call Controller Task for control Update
		adc_process_task(); // Call ADC Task for sensors update
		state_machine_task(); // Here add the other concurrent process
#endif //TODO: probably remove 5apr15

#if 0 //for debugging the serial ID chips
		while(1)
		{
			read_serial_ids();
		}
#endif

		switch(sealShieldState)
		{
			case STATE_SS_IDLE:
				if (SS_DOOR_LATCHED) {
					gpio_set_pin_low(SEALSHIELD_DEBUG_LED);
					print_ssdbg("Door latch detected\r\n");
//					display_text(IDX_CLEAR);
					display_text(IDX_READY);
					sealShieldState = STATE_DOOR_LATCHED;
					firstTimeThrough = 1;
				}
				break;
				
			case STATE_DOOR_LATCHED:
				if (!SS_ACTION_PB) {
					print_ssdbg("Action push button press detected\r\n");
					sealShieldState = STATE_ACTION_PB_PRESSED;
					read_serial_ids();
				}
				break;
				
			case STATE_ACTION_PB_PRESSED:
				if (SS_ACTION_PB)
				{
					print_ssdbg("Action push button release detected\r\n");
					sealShieldState = STATE_ACTION_PB_RELEASED;	
					read_serial_ids();
				}
				break;
				
			case STATE_ACTION_PB_RELEASED:
				read_serial_ids();
				check_led_brd_side_lifetimes();
				check_shelves_for_devices();
				set_shelves_active_inactive();
				
				if (num_active_shelves() != 0) {
					sealShieldState = STATE_START_SANITIZE;	
					print_ssdbg("Start sanitizing\r\n");
					display_text(IDX_CLEAR);
					cpu_delay_ms(500, 8000000);
					display_text(IDX_CLEANING);
					cpu_delay_ms(3000, 8000000); //give display time to update, scroll all the way across
				}
				else {
					sealShieldState = STATE_START_CLEAN;
					print_ssdbg("Either no devices or shelves are past lifetime, charging devices\r\n");
//					display_text(IDX_CLEAR);
					display_text(IDX_READY);
				}
				break;
				
			case STATE_START_SANITIZE:
				sanitizeIdx = 0xFF; //this means not assigned yet
				for (int i = 0; i<NUM_SHELVES; i++) {
					if (shelfActive[i] == SHELF_ACTIVE) {
						shelfTimerInitSeconds[i] = calc_sanitize_time(i);
						
						led_shelf(i, LED_ON); //12apr15 turning the shelves on and leaving them on for the show. Use the timers I set up to cycle through the display text.
						
						if (sanitizeIdx == 0xFF)
						{
							sanitizeIdx = i; //set this to the first active shelf
						}
					}
					else {
						shelfTimerInitSeconds[i] = 0; //Don't run this shelf
					}
				}
				
				sealShieldState = STATE_SANITIZE_1;
				break;
				
			case STATE_SANITIZE_1:
/*
 * Code specifically for the show, just turn on one shelf at a time, we need work on the power supply to get it up to full power (1A)
 */

				display_text(IDX_CLEAR);
				cpu_delay_ms(500, 8000000); //half second
				
				if (shelfTimerInitSeconds[sanitizeIdx] != 0)
				{
					cpu_set_timeout(shelfTimerInitSeconds[sanitizeIdx], shelfTimerPtr[sanitizeIdx]); //cpu cycle counts and the pointer to the timer variable for this particular shelf
					switch (sanitizeIdx)
					{
						case 0:
							//								display_text(IDX_CLEAR);
							display_text(IDX_SHELF1);
							break;
						case 1:
							//								display_text(IDX_CLEAR);
							display_text(IDX_SHELF2);
							break;
						case 2:
							//								display_text(IDX_CLEAR);
							display_text(IDX_SHELF3);
							break;
						case 3:
							//								display_text(IDX_CLEAR);
							display_text(IDX_SHELF4);
							break;
					}

//12apr15 shelf st					led_shelf(sanitizeIdx, LED_ON);	//NOTE we need to be careful here, we need to be able to shut off the shelf LEDs the *instant* the door latch opens, this is important for safety
					//this means we need as little logic between turning the shelf on and turning it off so we can react as quickly as possible to the door latch
										
					sealShieldState = STATE_SANITIZE_2;
				}
				else
				{
					//Nothing on that shelf, go to the next shelf
					if (++sanitizeIdx >= NUM_SHELVES)
					{
						sanitizeIdx = 0; //12apr15 wrap around
					} 
				}

/*
 * End of code specifically for the show
 */				

				
#if 0 //we will use this code in the production unit but for the show do one shelf at a time
				anyShelvesStillSanitizing = 0;

				for (int i=0; i<NUM_SHELVES; i++) {
					if (shelfActive[i] == SHELF_ACTIVE)
					{
						if (!cpu_is_timeout(shelfTimerPtr[i])) {
							anyShelvesStillSanitizing++;
						}
						else {
							led_shelf(i, LED_OFF);
						}
						
						if (!SS_DOOR_LATCHED) //check this often in the sanitize state, need to kill the shelves the instant the door opens for safety
						{
							door_latch_open_kill_all_shelves();
							display_text(IDX_CLEAR);
							cpu_delay_ms(1000, 8000000);
							display_text(IDX_DIRTY);
							display_text(IDX_DIRTY);
						}
					}
				}
				
				if (!anyShelvesStillSanitizing) {
					cpu_stop_timeout(shelfTimerPtr[0]);
					cpu_stop_timeout(shelfTimerPtr[1]);
					cpu_stop_timeout(shelfTimerPtr[2]);
					cpu_stop_timeout(shelfTimerPtr[3]);
					sealShieldState = STATE_START_CLEAN;
					print_ssdbg("All shelves clean\r\n");
//					display_text(IDX_CLEAR);
					display_text(IDX_CLEAN);

				}
#endif				
				break;
				
			case STATE_SANITIZE_2:
				if (cpu_is_timeout(shelfTimerPtr[sanitizeIdx])) {
//12apr15 leave shelves on indefinitely for the show					led_shelf(sanitizeIdx, LED_OFF);
					cpu_stop_timeout(shelfTimerPtr[sanitizeIdx]);
					print_ssdbg("Shelf clean\r\n");

					//All done, go to the next shelf
					if (++sanitizeIdx >= NUM_SHELVES)
					{
						sanitizeIdx = 0;
						sealShieldState = STATE_SANITIZE_1; //that was the last shelf, get out of sanitizing
					}
					else
					{
						sealShieldState = STATE_SANITIZE_1; //more shelves to clean possibly
					}
				}
				break;
				
			case STATE_START_CLEAN:
				display_text(IDX_CLEAN);
				sealShieldState = STATE_CLEAN;
				cpu_set_timeout((7*SS_ONE_SECOND), &timerClean); //Fixed for now just to make the display look good, show the string and let it wrap once TODO: make this what it needs to be
				break;	
				
			case STATE_CLEAN:
				if (cpu_is_timeout(&timerClean)) {
					cpu_stop_timeout(&timerClean);
					sealShieldState = STATE_ACTION_PB_RELEASED;	
					print_ssdbg("Start sanitizing\r\n");

				}
				break;
				
			case STATE_SHUTDOWN_PROCESSES:
				//Shutdown all processes that could harm the user or equipment if the door is open
				led_shelf(0, LED_OFF);
				led_shelf(1, LED_OFF);
				led_shelf(2, LED_OFF);
				led_shelf(3, LED_OFF);
				sealShieldState = STATE_SS_IDLE;
				break;
		} //switch(sealShieldState)
		
		/*
		 * This check overrides everything going on in the state machine, if the user opens the door,
		 * shut down all processes for safety
		 */
		if (!SS_DOOR_LATCHED) {
		
			if (firstTimeThrough)
			{
				door_latch_open_kill_all_shelves();

				display_text(IDX_CLEAR);
				cpu_delay_ms(500, 8000000);
				switch (sealShieldState)
				{
					case STATE_SANITIZE_1:
					case STATE_SANITIZE_2:
						display_text(IDX_DIRTY);
						break;
					
					default:
						display_text(IDX_CLEAN);
						break;
				}

				sealShieldState = STATE_SHUTDOWN_PROCESSES;
				print_ssdbg("Door latch opened, shutting down all processes\r\n");
				firstTimeThrough = 0;
				
			}
		} //if (!SS_DOOR_LATCHED)
		
		if (cpu_is_timeout(&timerDebugLed))
		{
			cpu_stop_timeout(&timerDebugLed);
			cpu_set_timeout((SS_ONE_SECOND/2), &timerDebugLed);
			gpio_toggle_pin(SEALSHIELD_DEBUG_LED);
		}
	} //while(true)
} //main
