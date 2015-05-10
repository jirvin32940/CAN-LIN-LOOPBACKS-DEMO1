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
#include "ec_print_funcs.h" //8apr15 changed from print_funcs.h
#include "flashc.h"
#include "adcifa.h"
#include "twim.h"
#include "conf_pca9952.h" //6apr15
#include "pca9952.h" //7apr15
#include "conf_eclave.h"	//8apr15
#include "cycle_counter.h"		//8apr15	
#include "usart.h"				//9apr15
#include "serial_id_ds2411.h"	//9apr15
#include "flashc.h"				//2may15
#include "string.h"


unsigned char read_usage_struct(unsigned char sel);
unsigned char test_flash(unsigned char sel);
void add_new_led_board_sides_to_usage(unsigned char sel);
unsigned char calc_usage_csum(unsigned char sel);
void copy_usage_to_usage(unsigned char dst, unsigned char src);
void write_usage_to_flash(unsigned char sel);
void load_usage_indeces(unsigned char sel);


unsigned char pingPong; //used to toggle between 2 regions of flash

#define NO_LED_BOARD_PRESENT 0xFF
#define NUM_LED_BOARDS			5
#define NUM_LED_BOARD_SIDES		8
#define NUM_SHELVES				4	//Shelf 0 is board 0 bottom + board 1 top
									//Shelf 1 is board 1 bottom + board 2 top
									//Shelf 2 is board 2 bottom + board 3 top
									//Shelf 3 is board 3 bottom + board 4 top
									//Board 0 upper side and board 4 lower side are not used


unsigned char usageIdx[2][NUM_LED_BOARD_SIDES];


enum { BOTTOM, TOP};


typedef struct {
	
	unsigned char active;
	unsigned char tLedIdx;
	unsigned char bLedIdx;
	unsigned char devicesPresent;

} SHELF;


typedef struct {
	
	unsigned char idFamily;
	unsigned char id[6];
	unsigned char idcsum;
	unsigned char present;
	unsigned char shelfIdx;
	unsigned char uSideIdx;
	unsigned char lSideIdx;
	unsigned char uSideShelfIdx;
	unsigned char lSideShelfIdx;
	
} LEDBRD;
	
typedef struct {
	
	unsigned char sanitizeMinutes;
	unsigned char ushdwIdx;
	unsigned char maxUsageReached;
	unsigned char shelfIdx;
	unsigned char boardIdx;
	
} LEDBRDSIDE;		


SHELF shelf[NUM_SHELVES];
LEDBRD ledBrd[NUM_LED_BOARDS];
LEDBRDSIDE ledBrdSide[NUM_LED_BOARD_SIDES];

unsigned long sanitizeMinutes;
unsigned long tmpSanitizeMinutes;
unsigned long displayTimerSeconds;
t_cpu_time displayTimer;
t_cpu_time sanitizeTimer;
t_cpu_time oneMinuteTimer;
t_cpu_time cleanTimer;
t_cpu_time debugTimer;

unsigned char electroclaveState;



/*
 * NOTE: Don't let these structs exceed 2K bytes or they will overrun the flash areas they are assigned to.
 */ 

#define NUM_SETS_LED_BOARD_SIDES	12	//should be enough for the lifetime of the unit
#define LED_BOARD_SIDE_STRUCT_SIZE	10	//bytes


/* Data structure for serial ID and usage info */
typedef struct {
	
	unsigned char id[6];			//48 bits
	
	unsigned char maxUsageReached	:1;	//go/no-go flag
	unsigned char top_botn			:1; //top .=. 1, bottom .=. 0 side of the LED board (track them independently)
	unsigned char slotFilled		:1;
	unsigned char					:1;
	unsigned char					:1;
	unsigned char					:1;
	unsigned char					:1;
	unsigned char					:1;
	
	unsigned char hrs_thous : 4;	/* usage time: kinda BCD */
	unsigned char hrs_huns	: 4;
	unsigned char hrs_tens	: 4;
	unsigned char hrs_ones	: 4;
	unsigned char min_tens	: 4;
	unsigned char min_ones	: 4;
	
} SERIAL_ID_AND_USAGE; //10 bytes each



typedef struct {
		SERIAL_ID_AND_USAGE u[NUM_SETS_LED_BOARD_SIDES * NUM_LED_BOARD_SIDES];
		unsigned char		csum;
	} USAGE_SHADOW;

USAGE_SHADOW usageShdw[2];


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

#define ECLAVE_DOOR_LATCH	AVR32_PIN_PB30
#define ECLAVE_ACTION_PB	AVR32_PIN_PB31
#define ECLAVE_DEBUG_LED	AVR32_PIN_PD28
#define ECLAVE_PSUPPLY_ONn	AVR32_PIN_PA23
#define ECLAVE_LED_OEn		AVR32_PIN_PA22
#define ECLAVE_MFP			AVR32_PIN_PA21	//set to 1 for 1X, set to 0 for 4X

#define EC_DOOR_LATCHED (!gpio_get_pin_value(ECLAVE_DOOR_LATCH)) //12apr15 this is the correct sense for the equipment going to the show
#define EC_ACTION_PB	(!gpio_get_pin_value(ECLAVE_ACTION_PB)) //12apr15 this is the correct sense for the equipment going to the show


enum {
	SHELF_ACTIVE,
	SHELF_INACTIVE
};


void display_text(unsigned char idx);
void display_text(unsigned char idx)
{
	for (int i = 0; i<7; i++)
	{
		usart_putchar(DISPLAY_USART, ((unsigned char) ((*(cmdPtrArray[idx]+i)))));
	}
	
}

void chassis_error(void);
void chassis_error(void)
{
	display_text(IDX_ERROR);
	
	while(1); //catastrophic error, just hang TODO: allow technician interface to work here possibly
	
}

void init_io(void);
void init_io(void)
{
	uint32_t ioFlags;
	
	
	ioFlags = (GPIO_DIR_INPUT);
	gpio_configure_pin(ECLAVE_DOOR_LATCH, ioFlags);

	ioFlags = (GPIO_DIR_INPUT);
	gpio_configure_pin(ECLAVE_ACTION_PB, ioFlags);

	ioFlags = (GPIO_DIR_OUTPUT | GPIO_INIT_HIGH);
	gpio_configure_pin(ECLAVE_SERIAL_ID0, ioFlags);

	ioFlags = (GPIO_DIR_OUTPUT | GPIO_INIT_HIGH);
	gpio_configure_pin(ECLAVE_SERIAL_ID1, ioFlags);

	ioFlags = (GPIO_DIR_OUTPUT | GPIO_INIT_HIGH);
	gpio_configure_pin(ECLAVE_SERIAL_ID2, ioFlags);

	ioFlags = (GPIO_DIR_OUTPUT | GPIO_INIT_HIGH);
	gpio_configure_pin(ECLAVE_SERIAL_ID3, ioFlags);

	ioFlags = (GPIO_DIR_OUTPUT | GPIO_INIT_HIGH);
	gpio_configure_pin(ECLAVE_SERIAL_ID4, ioFlags);

	ioFlags = (GPIO_DIR_OUTPUT | GPIO_INIT_LOW);
	gpio_configure_pin(ECLAVE_DEBUG_LED, ioFlags);

	ioFlags = (GPIO_DIR_OUTPUT | GPIO_INIT_HIGH);
	gpio_configure_pin(ECLAVE_PSUPPLY_ONn, ioFlags);

	ioFlags = (GPIO_DIR_OUTPUT | GPIO_INIT_HIGH);
	gpio_configure_pin(ECLAVE_LED_OEn, ioFlags);

	ioFlags = (GPIO_DIR_OUTPUT | GPIO_INIT_LOW); //high=1x multiplier, low=4x multiplier 10apr15
	gpio_configure_pin(ECLAVE_MFP, ioFlags);

}


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

/* One serial ID chip per board */
void read_led_board_serial_ids(void);
void read_led_board_serial_ids(void)
{
	/*
	 * Check for LED board presence by issuing a reset to the serial ID chip and checking for a response.
	 */
	
	SetSpeed(1); //1==standard speed, not overdrive
	
	ledBrd[0].present = !OWTouchReset(0);
	ledBrd[1].present = !OWTouchReset(1);
	ledBrd[2].present = !OWTouchReset(2);
	ledBrd[3].present = !OWTouchReset(3);
	ledBrd[4].present = !OWTouchReset(4);
	
	for (int i=0; i<NUM_LED_BOARDS; i++)
	{
		if (ledBrd[i].present)
		{
			OWWriteByte(i, 0x33); //Read ID command
			
			ledBrd[i].idFamily = OWReadByte(i);
			
			for (int j=0; j<6; j++)
			{
				ledBrd[i].id[j] = OWReadByte(i);
			}
			
			ledBrd[i].idcsum = OWReadByte(i);
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
unsigned char check_led_brd_side_lifetime(unsigned char sideIdx);
unsigned char check_led_brd_side_lifetime(unsigned char sideIdx)
{
	unsigned char idx;
	unsigned int hours;
	float intensity; //TODO: needs to be float or dealt with differently
	
	/*
	 * Find the record for this board's serial ID number, and check the usage hours and see if we
	 *	are past the 2000 hour mark. If we are, this board is considered un-usuable until it is
	 *	refurbished. 
	 */
	
	idx = usageIdx[0][sideIdx];
	hours = (usageShdw[0].u[idx].hrs_thous * 1000) +
		(usageShdw[0].u[idx].hrs_huns * 100) +
		(usageShdw[0].u[idx].hrs_tens * 10) +
		(usageShdw[0].u[idx].hrs_ones);
		

/*
 * Since we have to calculate the hours to see if the shelf is valid, finish out the calculations for the sanitizing time also. We'll need it later.
 */
	intensity = ((0.00002 * hours * hours) - (0.0699 * hours) + 92.879);
		
	ledBrdSide[sideIdx].sanitizeMinutes = (20 * 100)/intensity; //Shortest sanitize time is 20 minutes. Sanitize time increases as LED intensity drops with usage. Sanitize time is around 49 minutes when usage is at 2000 hours.
	
	if (hours < 1999)
	{
		return LED_BOARD_SIDE_WITHIN_LIFETIME_LIMIT;
	}
	else
	{
		return LED_BOARD_SIDE_PAST_LIFETIME_LIMIT;
	}
}

/* Aggregate the information */
void check_led_brd_side_lifetimes(void);
void check_led_brd_side_lifetimes(void)
{
	for (int i=0; i<NUM_LED_BOARD_SIDES; i++)
	{
		ledBrdSide[i].maxUsageReached = !check_led_brd_side_lifetime(i);
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
		shelf[i].devicesPresent = check_shelf_for_devices(i);
	}
}

unsigned char numActiveShelves;

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
		shelf[i].active = SHELF_INACTIVE;
		
	}
	
	/* check shelf 0 */
	if (ledBrd[0].present &&
		ledBrd[1].present &&
		shelf[0].devicesPresent && 
		(!ledBrdSide[LED_BRD_0_BOT].maxUsageReached) &&
		(!ledBrdSide[LED_BRD_1_TOP].maxUsageReached) )
	{
		shelf[0].active = SHELF_ACTIVE;
		numActiveShelves++;
		print_ecdbg("Shelf 0 active\r\n");
	}
	
	/* check shelf 1 */
	
	if (ledBrd[1].present &&
		ledBrd[2].present &&
	shelf[1].devicesPresent &&
	(!ledBrdSide[LED_BRD_1_BOT].maxUsageReached) &&
	(!ledBrdSide[LED_BRD_2_TOP].maxUsageReached) )
	{
		shelf[1].active = SHELF_ACTIVE;
		numActiveShelves++;
		print_ecdbg("Shelf 1 active\r\n");
	}
	
	/* check shelf 2 */
	
	if (ledBrd[2].present &&
		ledBrd[3].present &&
	shelf[2].devicesPresent &&
	(!ledBrdSide[LED_BRD_2_BOT].maxUsageReached) &&
	(!ledBrdSide[LED_BRD_3_TOP].maxUsageReached) )
	{
		shelf[2].active = SHELF_ACTIVE;
		numActiveShelves++;
		print_ecdbg("Shelf 2 active\r\n");
	}
	
	/* check shelf 3 */
	
	if (ledBrd[3].present &&
		ledBrd[4].present &&
	shelf[3].devicesPresent &&
	(!ledBrdSide[LED_BRD_3_BOT].maxUsageReached) &&
	(!ledBrdSide[LED_BRD_4_TOP].maxUsageReached) )
	{
		shelf[3].active = SHELF_ACTIVE;
		numActiveShelves++;
		print_ecdbg("Shelf 3 active\r\n");
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
	STATE_EC_IDLE,
	STATE_DOOR_LATCHED,
	STATE_ACTION_PB_PRESSED,
	STATE_ACTION_PB_RELEASED,
	STATE_START_SANITIZE,
	STATE_SANITIZE,
	STATE_START_CLEAN,
	STATE_CLEAN,
	STATE_SHUTDOWN_PROCESSES
};

unsigned long calc_sanitize_time(unsigned char shelfIdx);
unsigned long calc_sanitize_time(unsigned char shelfIdx)
{
	unsigned char uSideMinutes, lSideMinutes, minutes, boardIdx, sideIdx;
	
	boardIdx = shelf[shelfIdx].tLedIdx;							//top board in the shelf
	sideIdx = ledBrd[boardIdx].lSideIdx;						//lower side of the top board
	lSideMinutes = ledBrdSide[sideIdx].sanitizeMinutes;
	

	boardIdx = shelf[shelfIdx].bLedIdx;							//bottom board in the shelf					
	sideIdx = ledBrd[boardIdx].uSideIdx;						//upper side of the bottom board
	uSideMinutes = ledBrdSide[sideIdx].sanitizeMinutes;

	minutes = (uSideMinutes >= lSideMinutes) ? uSideMinutes : lSideMinutes; //choose the sanitize time for the more worn-out leds
	
	return (minutes * 60 * cpu_ms_2_cy(1000, 80000000));
	
}


void door_latch_open_kill_all_shelves(void);
void door_latch_open_kill_all_shelves(void)
{
	led_shelf(0, LED_OFF);
	led_shelf(1, LED_OFF);
	led_shelf(2, LED_OFF);
	led_shelf(3, LED_OFF);
}


/*
 * 2 copies: one each for alternating minutes.
 * Need these areas of flash to erase and be written independently.
 * It's very easy for the power to get shut down during the 
 * one minute updates while the unit is sanitizing. Keeping these
 * 2 areas of flash erasing and updating independently ensures that at
 * least one buffer is intact if the other buffer gets corrupted.
 */



//! NVRAM data structure located in the flash array.
#if defined (__GNUC__)
__attribute__((__section__(".flash_nvram0")))
#endif
static SERIAL_ID_AND_USAGE serialIdAndUsageFlashZero[NUM_SETS_LED_BOARD_SIDES * NUM_LED_BOARD_SIDES]
#if defined (__ICCAVR32__)
@ "FLASH_NVRAM0"
#endif
;

#if defined (__GNUC__)
__attribute__((__section__(".flash_nvram1")))
#endif
static SERIAL_ID_AND_USAGE serialIdAndUsageFlashOne[NUM_SETS_LED_BOARD_SIDES * NUM_LED_BOARD_SIDES]
#if defined (__ICCAVR32__)
@ "FLASH_NVRAM1"
#endif
;

unsigned char usage_idx(unsigned char sel, unsigned char * idPtr, unsigned char top_botn);
unsigned char usage_idx(unsigned char sel, unsigned char * idPtr, unsigned char top_botn)
{
	for (unsigned char i=0; i<(NUM_SETS_LED_BOARD_SIDES * NUM_LED_BOARD_SIDES); i++)
	{
		if ((strstr((char*)idPtr, (char*)(usageShdw[sel].u[i].id))) && (usageShdw[sel].u[i].top_botn == top_botn))
		{
			return (i); //Found a match!
		}
	}
	
	return NO_LED_BOARD_PRESENT;

}

void load_usage_indeces(unsigned char sel)
{
	usageIdx[sel][0] = usage_idx(sel, &ledBrd[0].id[0], BOTTOM);
	usageIdx[sel][1] = usage_idx(sel, &ledBrd[1].id[0], TOP);
	usageIdx[sel][2] = usage_idx(sel, &ledBrd[1].id[0], BOTTOM);
	usageIdx[sel][3] = usage_idx(sel, &ledBrd[2].id[0], TOP);
	usageIdx[sel][4] = usage_idx(sel, &ledBrd[2].id[0], BOTTOM);
	usageIdx[sel][5] = usage_idx(sel, &ledBrd[3].id[0], TOP);
	usageIdx[sel][6] = usage_idx(sel, &ledBrd[3].id[0], BOTTOM);
	usageIdx[sel][7] = usage_idx(sel, &ledBrd[4].id[0], TOP);
}

enum{CHECKSUM_INVALID, CHECKSUM_VALID};

unsigned char read_usage_struct(unsigned char sel)
{
	unsigned char tmpCsum;
	
	if (sel == 0)
	{
		memcpy(&usageShdw[0],serialIdAndUsageFlashZero, sizeof(usageShdw[0]));
	}
	else
	{
		memcpy(&usageShdw[1],serialIdAndUsageFlashOne, sizeof(usageShdw[1]));
	}
	
	tmpCsum = calc_usage_csum(sel);
	
	if (tmpCsum == usageShdw[sel].csum)
	{
		return CHECKSUM_VALID;
	}
	else
	{
		return CHECKSUM_INVALID;
	}
}

enum {SUCCESS, ERROR};

unsigned char test_flash(unsigned char sel)
{
	volatile void* memPtr;
	unsigned char pattern[4] = {0xFF, 0x00, 0xAA, 0x55}, ubyte;
	unsigned char *ubPtr;
	unsigned long memSize;
	
	memSize = sizeof(usageShdw[sel]);
	
	if (sel == 0)
	{
		memPtr = &serialIdAndUsageFlashZero;
	}
	else
	{
		memPtr = &serialIdAndUsageFlashOne;
	}

	for (unsigned char i=0; i<4; i++) //4 patterns to test
	{
		flashc_memset(memPtr, pattern[i], 8, memSize, true);
	
		ubPtr = (unsigned char*) memPtr;
		for (unsigned long j=0; j<memSize; j++)
		{
			ubyte = (*ubPtr);
			if (ubyte != pattern[i])
			{
				return ERROR;
			}
			ubPtr++;
		}
	}
	
	return SUCCESS;
}


#define USAGE_FULL 0xFF

unsigned char find_first_open_usage_slot(unsigned char sel);
unsigned char find_first_open_usage_slot(unsigned char sel)
{
	for (unsigned int i=0; i<(NUM_SETS_LED_BOARD_SIDES * NUM_LED_BOARD_SIDES); i++)
	{
		if (!usageShdw[sel].u[i].slotFilled)
		{
			return i;
		}
	}
	
	return USAGE_FULL; //Error, no open slots
}

void add_new_led_board_sides_to_usage(unsigned char sel)
{
	unsigned char firstOpenSlot, slotAssignment, brdIdx, top_botn;
	
	//NOTE that load_usage_indeces() must have been run already for this function to work. 
	// i.e., usageIdx[][] must be populated.
	
	firstOpenSlot = find_first_open_usage_slot(sel);
	
	slotAssignment = firstOpenSlot;
	
	for (unsigned char i=0; i<NUM_LED_BOARD_SIDES; i++)
	{
		brdIdx = ledBrdSide[i].boardIdx;
		
		top_botn = (i%2) ? TOP : BOTTOM; //odd sides are top, even sides are bottom
		
		if ((ledBrd[brdIdx].present) && (usageIdx[sel][i] == NO_LED_BOARD_PRESENT)) //TODO: do I need the NO_LED_BOARD_PRESENT check? this should always be open at this point
		{
			strncpy((char*)&usageShdw[sel].u[slotAssignment].id[0], (char*)&ledBrd[brdIdx].id[0],6);
			
			usageShdw[sel].u[slotAssignment].top_botn = top_botn;
			
			usageShdw[sel].u[slotAssignment].slotFilled = 1;

			usageIdx[sel][i] = slotAssignment++;
		}
	}
}

unsigned char calc_usage_csum(unsigned char sel)
{
	unsigned char csum = 0;
	
/*
 * Lots of ways to checksum this struct, don't over-think it
 */

	for (unsigned char i=0; i<(NUM_SETS_LED_BOARD_SIDES * NUM_LED_BOARD_SIDES); i++)
	{
		csum += usageShdw[sel].u[i].hrs_thous;
		csum += usageShdw[sel].u[i].hrs_huns;
		csum += usageShdw[sel].u[i].hrs_tens;
		csum += usageShdw[sel].u[i].hrs_ones;
		csum += usageShdw[sel].u[i].min_tens;
		csum += usageShdw[sel].u[i].min_ones;
		
		csum += usageShdw[sel].u[i].id[0];
		csum += usageShdw[sel].u[i].id[1];
		csum += usageShdw[sel].u[i].id[2];
		csum += usageShdw[sel].u[i].id[3];
		csum += usageShdw[sel].u[i].id[4];
		csum += usageShdw[sel].u[i].id[5];
		
		csum += usageShdw[sel].u[i].maxUsageReached;
		csum += usageShdw[sel].u[i].top_botn;
		csum += usageShdw[sel].u[i].slotFilled;
	}
	
	return csum;
}

void copy_usage_to_usage(unsigned char dst, unsigned char src)
{
	memcpy(&usageShdw[dst], &usageShdw[src], sizeof(usageShdw[src]));
}

void write_usage_to_flash(unsigned char sel)
{
	if (sel == 0)
	{
		flashc_memcpy(serialIdAndUsageFlashZero, &usageShdw[0], sizeof(usageShdw[0]),true);
	}
	else
	{
		flashc_memcpy(serialIdAndUsageFlashOne, &usageShdw[0], sizeof(usageShdw[0]),true);
	}
}

unsigned long calc_usage_current_led_boards(unsigned char sel);
unsigned long calc_usage_current_led_boards(unsigned char sel)
{
	unsigned long hrs_thous = 0, 
		hrs_huns = 0, 
		hrs_tens = 0, 
		hrs_ones = 0, 
		min_tens = 0, 
		min_ones = 0;
		
	unsigned char idx;
	unsigned long retMinutes;
	
	for (unsigned char i=0; i<NUM_LED_BOARD_SIDES; i++)
	{
		if (usageIdx[sel][i] != NO_LED_BOARD_PRESENT)
		{
			idx = usageIdx[sel][i];
					
			hrs_thous += usageShdw[sel].u[idx].hrs_thous;
			hrs_huns += usageShdw[sel].u[idx].hrs_huns;
			hrs_tens += usageShdw[sel].u[idx].hrs_tens;
			hrs_ones += usageShdw[sel].u[idx].hrs_ones;
			min_tens += usageShdw[sel].u[idx].min_tens;
			min_ones += usageShdw[sel].u[idx].min_ones;
		}
	}
	
	retMinutes = (hrs_thous * 1000) + (hrs_huns * 100) + (hrs_tens * 10) + (hrs_ones);
	retMinutes *= 60;
	retMinutes += ((min_tens * 10) + (min_ones));
	
	return retMinutes;
}

void increment_ledBoard_usage_min(void);
void increment_ledBoard_usage_min(void)
{
	SERIAL_ID_AND_USAGE *tmp;
	unsigned char upperLEDboardIdx;
	unsigned char lowerLEDboardIdx;
	unsigned char upperLEDboardMinuteUsageIdx;
	unsigned char lowerLEDboardMinuteUsageIdx;
	
	for (unsigned char i=0; i<NUM_SHELVES; i++) //check every active shelf
	{
		if (shelf[i].active == SHELF_ACTIVE)
		{
			switch (i)
			{
				case 0:
					upperLEDboardIdx = 0;
					lowerLEDboardIdx = 1;
					break;
				case 1:
					upperLEDboardIdx = 1;
					lowerLEDboardIdx = 2;
					break;
				case 2:
					upperLEDboardIdx = 2;
					lowerLEDboardIdx = 3;
					break;
				case 3:
					upperLEDboardIdx = 3;
					lowerLEDboardIdx = 4;
					break;
			}
		
			upperLEDboardMinuteUsageIdx = usageIdx[0][upperLEDboardIdx];
			lowerLEDboardMinuteUsageIdx = usageIdx[0][lowerLEDboardIdx];
		
			for (unsigned char j=0; j<2; j++)
			{
				switch (j)
				{
					case 0:
						tmp = &usageShdw[0].u[upperLEDboardMinuteUsageIdx];
						break;
					case 1:
						tmp = &usageShdw[0].u[lowerLEDboardMinuteUsageIdx];
						break;
				}
			

				if (++(tmp->min_ones) > 9)
				{
					tmp->min_ones = 0;
				
					if (++(tmp->min_tens) > 5)
					{
						tmp->min_tens = 0;
					
						if (++(tmp->hrs_ones) > 9)
						{
							tmp->hrs_ones = 0;
						
							if (++(tmp->hrs_tens) > 9)
							{
								tmp->hrs_tens = 0;
							
								if (++(tmp->hrs_huns) > 9)
								{
									tmp->hrs_huns = 0;
								
									if (++(tmp->hrs_thous) > 1)
									{
										tmp->maxUsageReached = 1; //And...we're done. Reached 2000 hours.
									}
								}
							}
						}
					}
				}
			}
		}
	}
			
	write_usage_to_flash(pingPong);
	
	pingPong++;
	pingPong &= 1; //toggle between 0 (EVEN) and 1 (ODD)
}


void init_shelf_n_ledBrd_structs(void);
void init_shelf_n_ledBrd_structs(void)
{
	
	for (int i=0; i<NUM_SHELVES; i++)
	{
		shelf[i].active = 0;
		shelf[i].devicesPresent = 0;
	}
	
	shelf[0].tLedIdx = 0;
	shelf[0].bLedIdx = 1;
	shelf[1].tLedIdx = 1;
	shelf[1].bLedIdx = 2;
	shelf[2].tLedIdx = 2;
	shelf[2].bLedIdx = 3;
	shelf[3].tLedIdx = 3;
	shelf[3].bLedIdx = 4;
	
	for (int i=0; i<NUM_LED_BOARDS; i++)
	{
		ledBrd[i].present = 0;
	}
	
	ledBrd[0].uSideIdx = 0xFF;
	ledBrd[0].lSideIdx = 0;
	ledBrd[1].uSideIdx = 1;
	ledBrd[1].lSideIdx = 2;
	ledBrd[2].uSideIdx = 3;
	ledBrd[2].lSideIdx = 4;
	ledBrd[3].uSideIdx = 5;
	ledBrd[3].lSideIdx = 6;
	ledBrd[4].uSideIdx = 7;
	ledBrd[4].lSideIdx = 0xFF;

	ledBrd[0].uSideShelfIdx = 0xFF;
	ledBrd[1].uSideShelfIdx = 0;
	ledBrd[2].uSideShelfIdx = 1;
	ledBrd[3].uSideShelfIdx = 2;
	ledBrd[4].uSideShelfIdx = 3;
 
	ledBrd[0].lSideShelfIdx = 0;
	ledBrd[1].lSideShelfIdx = 1;
	ledBrd[2].lSideShelfIdx = 2;
	ledBrd[3].lSideShelfIdx = 3;
	ledBrd[4].lSideShelfIdx = 0xFF;
	

	for (int i=0; i<NUM_LED_BOARD_SIDES; i++)
	{
		ledBrdSide[i].maxUsageReached = 0;
		ledBrdSide[i].sanitizeMinutes = 0;
		ledBrdSide[i].ushdwIdx = 0xFF;
	}
	
	ledBrdSide[0].boardIdx = 0;
	ledBrdSide[1].boardIdx = 1;
	ledBrdSide[2].boardIdx = 1;
	ledBrdSide[3].boardIdx = 2;
	ledBrdSide[4].boardIdx = 2;
	ledBrdSide[5].boardIdx = 3;
	ledBrdSide[6].boardIdx = 3;
	ledBrdSide[7].boardIdx = 4;
	

	ledBrdSide[0].shelfIdx = 0;
	ledBrdSide[1].shelfIdx = 0;
	ledBrdSide[2].shelfIdx = 1;
	ledBrdSide[3].shelfIdx = 1;
	ledBrdSide[4].shelfIdx = 2;
	ledBrdSide[5].shelfIdx = 2;
	ledBrdSide[6].shelfIdx = 3;
	ledBrdSide[7].shelfIdx = 3;

}

void load_usageIdx_to_ledBrdSide(unsigned char sel);
void load_usageIdx_to_ledBrdSide(unsigned char sel)
{
	for (int i=0; i<NUM_LED_BOARD_SIDES; i++)
	{
		ledBrdSide[i].ushdwIdx = usageIdx[sel][i];
	}
}


void init_led_board_info(void);
void init_led_board_info(void)
{
	unsigned char usage0good, usage1good;
	unsigned int usage0cnt, usage1cnt;
	unsigned char newer, older, previouslyOlder;
	unsigned char good, bad, previouslyBad;
	
	init_shelf_n_ledBrd_structs();
	
	read_led_board_serial_ids();
	usage0good = read_usage_struct(0);
	usage1good = read_usage_struct(1);
	
	if (usage0good)
	{
		load_usage_indeces(0);
	}
	else
	{
		memset(&usageShdw[0], 0x00, sizeof(usageShdw[0]));
	}
	
	if (usage1good)
	{
		load_usage_indeces(1);
	}
	else
	{
		memset(&usageShdw[1], 0x00, sizeof(usageShdw[1]));
	}
	
	if ((!usage0good) && (!usage1good)) //Chassis is probably powering up for the first time
	{
		if (test_flash(0) == ERROR)
		{
			chassis_error();
		}
		if (test_flash(1) == ERROR)
		{
			chassis_error();
		}
		add_new_led_board_sides_to_usage(0);
		load_usageIdx_to_ledBrdSide(0);
		usageShdw[0].csum = calc_usage_csum(0);
		copy_usage_to_usage(1,0);
		write_usage_to_flash(0);
		write_usage_to_flash(1);
		pingPong = 0;
		
	}
	else if (usage0good && usage1good) //Both usage structs are good, find the newer one
	{
		usage0cnt = calc_usage_current_led_boards(0);
		usage1cnt = calc_usage_current_led_boards(1);
		
		if (usage0cnt >= usage1cnt)
		{
			newer = 0;
		}
		else
		{
			newer = 1;
		}
		older = newer ^ 1; //older is the opposite of newer
		
		if (test_flash(older) == ERROR)
		{
			chassis_error();
		}

		add_new_led_board_sides_to_usage(newer);
		load_usageIdx_to_ledBrdSide(newer);
		usageShdw[newer].csum = calc_usage_csum(newer);
		copy_usage_to_usage(older, newer);
		previouslyOlder = older;
		write_usage_to_flash(previouslyOlder);
		if (test_flash(newer) == ERROR)
		{
			chassis_error();
		}

		write_usage_to_flash(newer);
		pingPong = 0;
		
	}
	else //Only one usage struct is good, the other was probably corrupted during a power-down while sanitizing
	{
		if (usage0good)
		{
			good = 0;
		}
		else
		{
			good = 1;
		}
		bad = good ^ 1; //bad is the opposite of good
		
		if (test_flash(bad) == ERROR)
		{
			chassis_error();
		}
		
		add_new_led_board_sides_to_usage(good);
		load_usageIdx_to_ledBrdSide(good);
		usageShdw[good].csum = calc_usage_csum(good);
		copy_usage_to_usage(bad, good);
		previouslyBad = bad;
		write_usage_to_flash(previouslyBad);
		if (test_flash(good) == ERROR)
		{
			chassis_error();
		}
		
		write_usage_to_flash(good);
		pingPong = 0;
	}
	
}

unsigned char firstTimeThrough = 1;

/*! \brief Main File Section:
 *          - Initialization (CPU, TWI, Usart,...)
 */
int main(void)
{
	static unsigned char displayIdx = 0;
	
	// Initialize System Clock
	init_sys_clocks();

	init_io();
	

	// Initialize USART
	init_ecdbg_rs232(FPBA_HZ);
	init_display_rs232(FPBA_HZ);

	// Print Startup Message
	print_ecdbg("SEAL SHIELD DEMO \r\n Copyright (c) 2015 Technical Solutions Group, Inc.\r\n");
	display_text(IDX_READY);
	
	// Initialize ADC for bluesense channels which are used to see if there are any devices (phones, tablets, etc.) on the shelves
	adc_process_init();

	
	// Initialize Interrupts
	irq_initialize_vectors(); //TODO: probably remove 5apr15

	cpu_irq_enable();

	// Initialize TWI Interface
	twi_init();

	gpio_set_pin_high(ECLAVE_LED_OEn); //make sure outputs are disabled at the chip level
	PCA9952_init();
	
	electroclaveState = STATE_EC_IDLE;
	
	init_led_board_info();
	
	gpio_set_pin_low(ECLAVE_LED_OEn); //...and we are live!
	gpio_set_pin_low(ECLAVE_PSUPPLY_ONn); //turn the leds on first and then the power supply
	
	cpu_set_timeout(EC_ONE_SECOND/2, &debugTimer);


	// Main loop
	while (true) {

		switch(electroclaveState)
		{
			case STATE_EC_IDLE:
				if (EC_DOOR_LATCHED) {
					gpio_set_pin_low(ECLAVE_DEBUG_LED);
					print_ecdbg("Door latch detected\r\n");
//					display_text(IDX_CLEAR);
					display_text(IDX_READY);
					electroclaveState = STATE_DOOR_LATCHED;
					firstTimeThrough = 1;
				}
				break;
				
			case STATE_DOOR_LATCHED:
				if (!EC_ACTION_PB) {
					print_ecdbg("Action push button press detected\r\n");
					electroclaveState = STATE_ACTION_PB_PRESSED;
				}
				break;
				
			case STATE_ACTION_PB_PRESSED:
				if (EC_ACTION_PB)
				{
					print_ecdbg("Action push button release detected\r\n");
					electroclaveState = STATE_ACTION_PB_RELEASED;	
				}
				break;
				
			case STATE_ACTION_PB_RELEASED:
				check_led_brd_side_lifetimes();
				check_shelves_for_devices();
				set_shelves_active_inactive();
				
				if (num_active_shelves() != 0) {
					electroclaveState = STATE_START_SANITIZE;	
					print_ecdbg("Start sanitizing\r\n");
					display_text(IDX_CLEAR);
					cpu_delay_ms(500, 8000000);
					display_text(IDX_CLEANING);
					cpu_delay_ms(3000, 8000000); //give display time to update, scroll all the way across
				}
				else {
					electroclaveState = STATE_START_CLEAN;
					print_ecdbg("No shelves, no devices or shelves are past lifetime, charging devices\r\n");
//					display_text(IDX_CLEAR);
					display_text(IDX_READY);
				}
				break;
				
			case STATE_START_SANITIZE:
				displayIdx = 0xFF; //this means not assigned yet
				sanitizeMinutes = 0;
				for (int i = 0; i<NUM_SHELVES; i++) {
					if (shelf[i].active == SHELF_ACTIVE) {
						tmpSanitizeMinutes = calc_sanitize_time(i);
						
						if (tmpSanitizeMinutes > sanitizeMinutes)
						{
							sanitizeMinutes = tmpSanitizeMinutes;
						}
						
						led_shelf(i, LED_ON);
						
						if (displayIdx == 0xFF)
						{
							displayIdx = i; //set this to the first active shelf if this is the first active shelf encountered
						}
					}
				}
				
				displayTimerSeconds = cpu_ms_2_cy(8000, 8000000); //8 seconds per "shelf" display is enough time for the text to scroll twice
				cpu_set_timeout(displayTimerSeconds, &displayTimer);
				
				cpu_set_timeout((sanitizeMinutes * 60 * cpu_ms_2_cy(1000, 8000000)), &sanitizeTimer);
				
				cpu_set_timeout((60 * cpu_ms_2_cy(1000,8000000)), &oneMinuteTimer); //one minute for the usage statistics

				display_text(IDX_CLEAR);
				cpu_delay_ms(500, 8000000); //half second TODO: figure out why this is here and get rid of it, don't like to just hang for no reason, especially when we need to be monitoring the door latch
				
				electroclaveState = STATE_SANITIZE;
				
				break;
				
			case STATE_SANITIZE:
				/*
    			 * Manage the display
				 */
				if (cpu_is_timeout(&displayTimer))
				{
					cpu_stop_timeout(&displayTimer);
					switch (displayIdx)
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
					
					while (1)
					{
						if (++displayIdx >= NUM_SHELVES)
						{
							displayIdx = 0; //12apr15 wrap around
						}
						
						if (shelf[displayIdx].active)
						{
							break; //this shelf is active, we don't need to look for another one
						}
						
					}

					cpu_set_timeout(displayTimerSeconds, &displayTimer); //8 seconds per shelf

					//NOTE we need to be careful here, we need to be able to shut off the shelf LEDs the *instant* the door latch opens, this is important for safety
					//this means we need as little logic between turning the shelf on and turning it off so we can react as quickly as possible to the door latch
				}

				/*
    			 * Manage storing usage statistics to flash
				 */
				if (cpu_is_timeout (&oneMinuteTimer))
				{
					cpu_stop_timeout (&oneMinuteTimer);
					
					increment_ledBoard_usage_min(); //increments usage minutes for active shelves only
					
					cpu_set_timeout(60000, &oneMinuteTimer); //one minute for the usage statistics
				}

				/*
    			 * Manage the sanitizer timer
				 */
				if (cpu_is_timeout(&sanitizeTimer)) {
					
					for (int i=0; i< NUM_SHELVES; i++)
					{
						led_shelf(i, LED_OFF); //turn off every shelf. (doesn't hurt to make sure that even non-active shelves are off.)
					}
					cpu_stop_timeout(&sanitizeTimer);
					print_ecdbg("Shelf clean\r\n");
				}
				break;
				
			case STATE_START_CLEAN:
				display_text(IDX_CLEAN);
				electroclaveState = STATE_CLEAN;
				cpu_set_timeout((20 * 60 * cpu_ms_2_cy(1000, 80000000)), &cleanTimer); //TODO: this time period will be parameterized from the technician UART interface
				break;	
				
			case STATE_CLEAN:
				if (cpu_is_timeout(&cleanTimer)) {
					cpu_stop_timeout(&cleanTimer);
					electroclaveState = STATE_ACTION_PB_RELEASED;	
					print_ecdbg("Start sanitizing\r\n");

				}
				break;
				
			case STATE_SHUTDOWN_PROCESSES:
				//Shutdown all processes that could harm the user or equipment if the door is open
				for (int i=0; i< NUM_SHELVES; i++)
				{
					led_shelf(i, LED_OFF); //turn off every shelf. (doesn't hurt to make sure that even non-active shelves are off.)
				}
				electroclaveState = STATE_EC_IDLE;
				break;
		} //switch(electroclaveState)
		
		/*
		 * This check overrides everything going on in the state machine, if the user opens the door,
		 * shut down all processes for safety
		 */
		if (!EC_DOOR_LATCHED) {
		
			if (firstTimeThrough)
			{
				door_latch_open_kill_all_shelves();

				display_text(IDX_CLEAR);
				cpu_delay_ms(500, 8000000);
				switch (electroclaveState)
				{
					case STATE_SANITIZE:
						display_text(IDX_DIRTY);
						break;
					
					default:
						display_text(IDX_CLEAN);
						break;
				}

				electroclaveState = STATE_SHUTDOWN_PROCESSES;
				print_ecdbg("Door latch opened, shutting down all processes\r\n");
				firstTimeThrough = 0;
				
			}
		} //if (!EC_DOOR_LATCHED)
		
		if (cpu_is_timeout(&debugTimer))
		{
			cpu_stop_timeout(&debugTimer);
			cpu_set_timeout((EC_ONE_SECOND/2), &debugTimer);
			gpio_toggle_pin(ECLAVE_DEBUG_LED);
		}
	} //while(true)
	
} //main
