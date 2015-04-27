/**
 * \file
 *
 * \brief Seal Shield Electroclave device sanitizer
 * based on Atmel's AVR UC3C CAN-LIN Loopback Demo
 *
 * Copyright (c) 2011-2014 Atmel Corporation. All rights reserved.
 *
 * Copyright (c) 2015 Seal Shield. All rights reserved.
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
 *
 * \section compilinfo Compilation Information
 * This software is written for GNU GCC for AVR32 and for IAR Embedded Workbench
 * for Atmel AVR32. Other compilers may or may not work.
 *
 * \section deviceinfo Device Information
 * All AVR32 AT32UC3C devices can be used.
 *
 * \section contactinfo Contact Information
 * For further information, visit
 * <A href="http://www.atmel.com/avr">Atmel AVR</A>.\n
 */
 /**
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
 */

#include "compiler.h"
#include "power_clocks_lib.h"
#include "gpio.h"
#include "ss_print_funcs.h" //8apr15 changed from print_funcs.h
#include "flashc.h"
#include "adcifa.h"
#include "twim.h"
#include "conf_pca9952.h" //6apr15
#include "pca9952.h" //7apr15
#include "conf_sealshield.h"	//8apr15
#include "cycle_counter.h"		//8apr15	
#include "usart.h"				//9apr15
#include "serial_id_ds2411.h"	//9apr15

/*
 * ADC reading storage: for device detection
 */
int16_t bluesense_buf[4];


/*
 * Commands for LED display: we can only display the strings provided for by the display
 */

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
	&CMD_CLEAR[0]
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


void display_text(unsigned char idx);
void display_text(unsigned char idx)
{
	for (int i = 0; i<7; i++)
	{
		usart_putchar(DISPLAY_USART, ((unsigned char) ((*(cmdPtrArray[idx]+i)))));
	}
	
}

void init_io(void);
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
void read_serial_ids(void);
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
unsigned char check_led_brd_side_lifetime(unsigned char ledBrdPosition);
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
void check_led_brd_side_lifetimes(void);
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
//26apr15	.reference_source         = ADCIFA_ADCREF0, // Reference Source
	.reference_source		  = ADCIFA_ADCREF,  // Reference Source 26apr15
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
//26apr15	ADCIFA_SOCB_ALLSEQ,
	ADCIFA_SOCB_SINGLECONV, //26apr15
	ADCIFA_SH_MODE_OVERSAMP,
	ADCIFA_HWLA_NOADJ,
	ADCIFA_SA_NO_EOS_SOFTACK
};

int16_t adc_values_seq0;
adcifa_sequencer_conversion_opt_t
adcifa_sequence_conversion_opt_seq0_shelf[4] = {
	{
		INPUT1_ADC_INP,
		INPUT1_ADC_INN,
		ADCIFA_SHG_8
	},
	{
		INPUT2_ADC_INP,
		INPUT2_ADC_INN,
		ADCIFA_SHG_8
	},
	{
		INPUT3_ADC_INP,
		INPUT3_ADC_INN,
		ADCIFA_SHG_8
	},
	{
		INPUT4_ADC_INP,
		INPUT4_ADC_INN,
		ADCIFA_SHG_8
	}
};

volatile avr32_adcifa_t *adcifa = &AVR32_ADCIFA; // ADCIFA IP registers address

int16_t adc_process_task(unsigned char shelfIdx);
int16_t adc_process_task(unsigned char shelfIdx)
{
	// Configure ADCIFA sequencer 0 for this particular shelf
	adcifa_configure_sequencer(adcifa, 0, &adcifa_sequence_opt,
		&adcifa_sequence_conversion_opt_seq0_shelf[shelfIdx]);

	// Start ADCIFA sequencer 0
	adcifa_start_sequencer(adcifa, 0);

	// Get Values from sequencer 0
	while(1)
	{
		//TODO: need a timeout here and error handling in case the ADC gets stuck for some reason
		
		if (adcifa_get_values_from_sequencer(adcifa, 0, &adcifa_sequence_opt, &adc_values_seq0) == ADCIFA_STATUS_COMPLETED) 
		{
			bluesense_buf[shelfIdx] = adc_values_seq0;
			return bluesense_buf[shelfIdx];
		}
	}
}




enum {
	NO_DEVICES_PRESENT,
	DEVICES_PRESENT
};

unsigned char check_shelf_for_devices(unsigned char shelfPosition);
unsigned char check_shelf_for_devices(unsigned char shelfPosition)
{
	U16 bluesense;
	
	led_shelf(shelfPosition, LED_ON); //TODO: do we finish this task fast enough to not check the door latch in here? Can't have LEDs on if the door opens
	
	cpu_delay_ms(50, 8000000);
		
	//Read bluesense for this shelf
	bluesense = 0;
	bluesense = adc_process_task(shelfPosition);

	led_shelf(shelfPosition, LED_OFF);
	

	if (bluesense > 0x800)
	{
		return DEVICES_PRESENT;
	}
	else
	{
		return NO_DEVICES_PRESENT;
	}
}

void check_shelves_for_devices(void);
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

void set_shelves_active_inactive(void);
void set_shelves_active_inactive(void)
{
	numActiveShelves = 0;
	
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

	unsigned char tmp1, tmp2, tmp3, tmp4;

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

unsigned char num_active_shelves(void);
unsigned char num_active_shelves(void)
{
	return numActiveShelves;
}


/*! \brief Initializes the MCU system clocks.
 */
/*
 * Using RC8M (internal 8MHz)
 */
void init_sys_clocks(void);
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

/*! \brief ADC Process Init
 *
 *
 */
void adc_process_init(void);
void adc_process_init(void)
{
	// GPIO pin/adc-function map.
	static const gpio_map_t ADCIFA_GPIO_MAP = {
//26apr15 using internal ref now		{AVR32_ADCREF0_PIN,AVR32_ADCREF0_FUNCTION},
		{AVR32_ADCREFP_PIN,AVR32_ADCREFP_FUNCTION},
		{AVR32_ADCREFN_PIN,AVR32_ADCREFN_FUNCTION},
		{INPUT1_ADC_PIN, INPUT1_ADC_FUNCTION},
		{INPUT2_ADC_PIN, INPUT2_ADC_FUNCTION},
		{INPUT3_ADC_PIN, INPUT3_ADC_FUNCTION},
		{INPUT4_ADC_PIN, INPUT4_ADC_FUNCTION}
	};

	// Assign and enable GPIO pins to the ADC function.
	gpio_enable_module(ADCIFA_GPIO_MAP,
			sizeof(ADCIFA_GPIO_MAP) / sizeof(ADCIFA_GPIO_MAP[0]));

	// Get ADCIFA Factory Configuration
	adcifa_get_calibration_data(adcifa, &adc_config_t);

	// Configure ADCIFA core
	adcifa_configure(adcifa, &adc_config_t, 8000000);

}

/*! \brief ADC Process Task
 *
 *
 */
/*! \brief TWI Initialization for QTouch Controller
 *
 *
 */
static void twi_init(void);
static void twi_init(void)
{
	const gpio_map_t PCA9952_TWI_GPIO_MAP = {
		{PCA9952_TWI_SCL_PIN, PCA9952_TWI_SCL_FUNCTION},
		{PCA9952_TWI_SDA_PIN, PCA9952_TWI_SDA_FUNCTION}
	};

	twi_options_t PCA9952_TWI_OPTIONS = { //7apr15 make this *not* a const so we can change it and rerun twi_master_init() if necessary
		.pba_hz = FPBA_HZ,
		.speed = PCA9952_TWI_MASTER_SPEED,
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

unsigned long calc_sanitize_time(unsigned char shelfIdx);
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


void door_latch_open_kill_all_shelves(void);
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
