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
#include "stdio.h"
#include "sysclk.h"
#include <pll.h>


unsigned char read_usage_struct(unsigned char sel);
unsigned char test_flash(unsigned char sel);

#ifdef SERIAL_ID_AND_ALL_USAGE_COMBINED
void add_new_led_board_sides_to_usage(unsigned char sel);
#else
void add_new_led_board_sides_to_usage(void);
#endif

unsigned char calc_usage_csum(unsigned char sel);
void copy_usage_to_usage(unsigned char dst, unsigned char src);
void write_usage_to_flash(unsigned char sel);

#if 0 //SERIAL_ID_AND_ALL_USAGE_COMBINED
void load_usage_indeces(unsigned char sel);
#endif

void load_usage_indeces(void);



unsigned char minute_count(unsigned char * pMinuteBits);
void reset_minutes(unsigned char * pMinuteBits);
unsigned char inc_minutes(unsigned char * pMinuteBits);

unsigned char firstTimeThroughDoorLatch = 1;
unsigned char firstTimeThroughPCA9952 = 1;


unsigned char minPingPong; //used to toggle between 2 buffers each minute
unsigned char hourPingPong; //used to toggle between 2 regions of flash each hour

#define NO_LED_BOARD_PRESENT 0xFF
#define NUM_LED_BOARDS			5
#define NUM_LED_BOARD_SIDES		8
#define NUM_SHELVES				4	//Shelf 0 is board 0 bottom + board 1 top
									//Shelf 1 is board 1 bottom + board 2 top
									//Shelf 2 is board 2 bottom + board 3 top
									//Shelf 3 is board 3 bottom + board 4 top
									//Board 0 upper side and board 4 lower side are not used

#ifdef SERIAL_ID_AND_ALL_USAGE_COMBINED
unsigned char usageIdx[2][NUM_LED_BOARD_SIDES];
#else
unsigned char usageIdx[NUM_LED_BOARD_SIDES];
#endif 

enum { BOTTOM, TOP};


typedef struct {
	
	unsigned char active;
	unsigned char tLedIdx;
	unsigned char bLedIdx;
	unsigned char devicesPresent;
	unsigned char present;

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


//this is the old way 24may15 #define SERIAL_ID_AND_ALL_USAGE_COMBINED
#ifdef SERIAL_ID_AND_ALL_USAGE_COMBINED

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
	
	unsigned long minutes;	
	unsigned char minuteBits[8];	//flip one bit from 1 to 0 each minute, saves us from having to erase flash every minute, we will only need to erase flash once per hour
	
} SERIAL_ID_AND_USAGE; //10 bytes each


typedef struct {
		SERIAL_ID_AND_USAGE u[NUM_SETS_LED_BOARD_SIDES * NUM_LED_BOARD_SIDES];
		unsigned long		totalSanitationMinutes;
		unsigned long		totalSanitationCycles;		
		unsigned char		csum;
	} USAGE_SHADOW;

USAGE_SHADOW usageShdw[2];

#else //new scheme

/*
 * CHASSIS AND BOARD STATISTICS STORED IN FLASH
 *
 * Using the bottom-most flash inside the Atmel processor.
 * Processor flash is organized as 128 byte sectors, and rated for 100,000 erase cycles.
 *
 *	DATA										UPDATE FREQUENCY
 * --------------------------------------------------------------
 * -Chassis sanitation cycles					Incremented on start of each sanitation cycle.
 * -Chassis sanitation minutes					Incremented each minute of each sanitation cycle.
 * -Serial ID and usage in hours plus flags		Serial IDs for each LED board logged when a board is plugged into the chassis.
 *												An entry is maintained for the board's top side, and another entry for the bottom side
 *												 *if* the board is not in an outer slot (0 or 4). A board is slot 0 will only have the 
 *												 bottom side tracked, and a board in slot 4 will only have its top side tracked. (If the board
 *												 is later moved to another slot, the second side will also start being tracked at that point.)
 *												Each LED board side's sanitation usage hours are tracked in this structure. This structure
 *												has room for 96 entries: 12 complete sets of 8 LED board sides which should be more than enough to  
												 cover the life of the product.
 *												This structure also tracks whether this slot in the structure has been filled, and whether
 *												max hour usage (2000 hours) has been reached.
 * -Usage in minutes							Similar structure to the serial ID and usage structure, except here we only track usage minutes.
 *												The reason for breaking them out separately is because there are potentially 96 entries that need
 *												to be updated once per minute, and since there is no guarantee that the minutes will roll over to
 *												hours at the same time for each of the 96 entries, we need to create lots of buffers for this struct
 *												to make sure we don't exceed the flash max erase cycles of 100,000.
 *
 */

/*
 * CHASSIS SANITATION CYCLES
 * 
 * Max value: (96 LED board sides / 2 boards per slot) * 2000 hours max per LED board * 3 cycles max per hour (20 minutes @aging of 0 or 100% intensity) = 288,000 = 0x46500
 * Bits required for max value: 19
 * Number of bits of checksum per entry, rounds up to the nearest byte - 24 - 19 bits = 5 bits for checksum. 24 bits = 3 bytes.
 * Number of buffers required for wear-leveling (100,000 erase cycle flash) - (288,000 / 100,000) =  ~3
 * Need at least 2 sectors in case one is in the middle of an update when the power goes down, so we will do 2 entries per sector for a total of 4 entries even though we only need 3.
 *
 */

typedef struct {
	
	unsigned int	cycles	: 19;
	unsigned int	csum	: 5;
	
}CHASSIS_SANITATION_CYCLES;

CHASSIS_SANITATION_CYCLES sanc;

#define NUM_SAN_CYCLE_BUFS_PER_SECTOR	2
#define NUM_SAN_CYCLE_BUFS_SECTORS		2	//we need at least two sectors so that if one is in the middle of an update when the power goes down another is still in tact

unsigned int sanCycleFlashIdx = 0;


/*
 * CHASSIS SANITATION MINUTES
 * 
 * Max value: (96 LED board sides / 2 boards per slot) * 2000 hours max per LED board * 60 minutes per hour = 5,760,000 = 0x57E400
 * Bits required for max value: 23
 * Number of bits of checksum per entry, rounds up to the nearest byte - 32 - 23 bits = 9 bits for checksum. 32 bits = 4 bytes.
 * Number of buffers required for wear-leveling (100,000 erase cycle flash) - (5,760,000 / 100,000) =  ~58
 * Need at least 2 sectors in case one is in the middle of an update when the power goes down, so we will do 58/2=29 entries per sector.
 *
 */

typedef struct {
	
	unsigned int	mins	: 23;
	unsigned int	csum	: 8;
	unsigned int			: 1;
	
}CHASSIS_SANITATION_MINUTES;

CHASSIS_SANITATION_MINUTES sanm;

#define NUM_SAN_MIN_BUFS_PER_SECTOR		29
#define NUM_SAN_MIN_BUFS_SECTORS		2	//we need at least two sectors so that if one is in the middle of an update when the power goes down another is still in tact

unsigned int sanMinFlashIdx = 0;


/*
 * SERIAL ID PLUS USAGE IN HOURS PLUS FLAGS
 * 
 * 96 LED board sides * 62 bit structure =  744 bytes per set = 744 bytes/128 bytes per sector = ~6 sectors per set.
 * Number of buffers required for wear-leveling (100,000 erase cycle flash) - (96 LED board sides *2000 hours of sanitization / 100,000 erase cycles) =  ~2 sets => 2 sets * 6 sectors = 12 sectors.
 *
 */

typedef struct {
	
	unsigned char id[6];					//6 bytes - 48 bits

	unsigned int  hours				: 11;
	
	unsigned int top_botn			: 1;	//top .=. 1, bottom .=. 0 side of the LED board (track them independently)
	unsigned int maxUsageReached	: 1;	//go/no-go flag
	unsigned int slotFilled			: 1;
	unsigned int					: 1;
	unsigned int					: 1;
	
} USAGE_SERIAL_ID_AND_HOURS;

typedef struct  
{
	USAGE_SERIAL_ID_AND_HOURS	u[(NUM_LED_BOARD_SIDES * NUM_SETS_LED_BOARD_SIDES)];
	unsigned char				csum;
}USAGE_SERIAL_ID_AND_HOURS_SET;

USAGE_SERIAL_ID_AND_HOURS_SET ush;

#define NUM_USAGE_HOURS_SECTORS_PER_BUF		6
#define NUM_USAGE_HOURS_BUFS_SECTORS		12

unsigned int ushFlashIdx = 0;



/*
 * USAGE MINUTES
 * 
 * 96 LED board sides * 6 bit structure  + 2 bit checksum =  96 bytes per set = 96/128 bytes per sector = ~1 sectors per set.
 * Number of buffers required for wear-leveling (100,000 erase cycle flash) - (96 LED board sides *2000 hours of sanitization * 60 minutes/hour / 100,000 erase cycles) =  116 sets = 116 sectors.
 *
 */


typedef struct  
{
	unsigned char	mins[(NUM_LED_BOARD_SIDES * NUM_SETS_LED_BOARD_SIDES)];
	unsigned char	csum;

} USAGE_MINS_SET;

USAGE_MINS_SET um;

#define NUM_USAGE_MINS_BUFS_PER_SECTOR		1
#define NUM_USAGE_MINS_BUFS_SECTORS			116

unsigned int umFlashIdx = 0;




#endif //old or new SERIAL_ID_AND_USAGE scheme

enum {SE_PASS, SE_FAIL};

typedef struct 
{
	unsigned char	topdrive;
	unsigned int	botdrive;	
	unsigned int	flashArea;		
	unsigned char	ledBrdSerialIdCsum;
	unsigned char	ledBrdSideMaxUsage;	
	unsigned char	usageStructsFull;
	
}SYSERR;

SYSERR sysErr;

#define BIT(x) (1<<(x))

void init_sysErr(void);
void init_sysErr(void)
{
	memset(&sysErr, 0x00, sizeof(sysErr)); //Init everything to "PASS"
}

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
	SHELF_INACTIVE,
	SHELF_ACTIVE
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
	print_ecdbg("Chassis error...shutting down.\r\n");
	
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


void print_ecdbg_num(unsigned int num);
void print_ecdbg_num(unsigned int num)
{
	char str[6];
	
	sprintf(str, "%d", num);	
	
	print_ecdbg(str);
}

/* One serial ID chip per board */
void read_led_board_serial_ids(void);
void read_led_board_serial_ids(void)
{
	/*
	 * Check for LED board presence by issuing a reset to the serial ID chip and checking for a response.
	 */
	
	SetSpeed(1); //1==standard speed, not overdrive 
	
	for (int i=0; i<NUM_LED_BOARDS; i++)
	{
		ledBrd[i].present = !OWTouchReset(i);
		if (ledBrd[i].present)
		{
			print_ecdbg("LED board detected in slot ");
			print_ecdbg_num(i);
			print_ecdbg("\r\n");
		}
	}
	
	if (ledBrd[0].present && ledBrd[1].present)
	{
		shelf[0].present = 1;
		
		print_ecdbg("Shelf 0 present\r\n");
	}
	if (ledBrd[1].present && ledBrd[2].present)
	{
		shelf[1].present = 1;
		print_ecdbg("Shelf 1 present\r\n");
	}
	if (ledBrd[2].present && ledBrd[3].present)
	{
		shelf[2].present = 1;
		print_ecdbg("Shelf 2 present\r\n");
	}
	if (ledBrd[3].present && ledBrd[4].present)
	{
		shelf[3].present = 1;
		print_ecdbg("Shelf 3 present\r\n");
	}
	

	for (int i=0; i<NUM_LED_BOARDS; i++)
	{
		unsigned char acc = 0;
		
		if (ledBrd[i].present)
		{
			OWWriteByte(i, 0x33); //Read ID command
			
			ledBrd[i].idFamily = OWReadByte(i);
			
			acc = crc8_add(0x00, ledBrd[i].idFamily);
			
			for (int j=0; j<6; j++)
			{
				ledBrd[i].id[j] = OWReadByte(i);
				acc = crc8_add(acc, ledBrd[i].id[j]);
			}
			
			ledBrd[i].idcsum = OWReadByte(i);
			
			if (acc != ledBrd[i].idcsum)
			{
				sysErr.ledBrdSerialIdCsum |= BIT(i); //SE_FAIL;
				ledBrd[i].present = 0; //crc8 wasn't valid for this ID chip, don't trust the board
				print_ecdbg("Invalid serial ID checksum.\r\n");
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
unsigned char check_led_brd_side_lifetime(unsigned char sideIdx);
unsigned char check_led_brd_side_lifetime(unsigned char sideIdx)
{
	unsigned char idx;
	unsigned int hours;
	float intensity;
	
	/*
	 * Find the record for this board's serial ID number, and check the usage hours and see if we
	 *	are past the 2000 hour mark. If we are, this board is considered un-usuable until it is
	 *	refurbished. 
	 */
	
	idx = ledBrdSide[sideIdx].ushdwIdx;
	
	#ifdef SERIAL_ID_AND_ALL_USAGE_COMBINED
	
	hours = (usageShdw[0].u[idx].minutes/60);
	if ((usageShdw[0].u[idx].minutes %60) > 30)
	{
		hours++;
	}
	
	#else
	
	hours = um.mins[idx]/60;
	if ((um.mins[idx]%60) > 30)
	{
		hours++;
	}
	
	#endif //SERIAL_ID_AND_ALL_USAGE_COMBINED
		

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
	unsigned char brdIdx;
	
	for (int i=0; i<NUM_LED_BOARD_SIDES; i++)
	{
		brdIdx = ledBrdSide[i].boardIdx;
		
		if (ledBrd[brdIdx].present)
		{
			ledBrdSide[i].maxUsageReached = !check_led_brd_side_lifetime(i);	
		}	
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
	
	cpu_delay_ms(50, EC_CPU_CLOCK_FREQ);
		
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
		if (shelf[i].present)
		{
			shelf[i].devicesPresent = check_shelf_for_devices(i);
			
			if (shelf[i].devicesPresent)
			{
				print_ecdbg("Devices detected on shelf ");
				print_ecdbg_num(i);
				print_ecdbg("\r\n");
			}
		}
	}
}


void print_pca9952_errors(unsigned char sideSel, unsigned char eflag0, unsigned char eflag1);
void print_pca9952_errors(unsigned char sideSel, unsigned char eflag0, unsigned char eflag1)
{
	unsigned char bit;
	
	switch (sideSel)
	{
		case TOP:
			print_ecdbg("PCA9952 Error(s) on TOPDRIVE ");
			
			for (int i=0; i<8; i++)
			{
				bit = (1 << i);
				if (bit & eflag0)
				{
					print_ecdbg_num(i);
					print_ecdbg(" ");
					sysErr.topdrive |= BIT(i); //SE_FAIL
				}
			}
			
			print_ecdbg("\r\n");
			
			if (eflag1 != 0)
			{
				print_ecdbg("ERROR on unused channels: PCA9952 - Controller board U7\r\n");
			}
			
			break;

		case BOTTOM:
			print_ecdbg("PCA9952 Error(s) on BOTDRIVE ");
			
			for (int i=0; i<8; i++)
			{
				bit = (1 << i);
				if (bit & eflag0)
				{
					print_ecdbg_num(i);
					print_ecdbg(" ");
					sysErr.botdrive |= BIT(i); //SE_FAIL;
				}
			}
			
			for (int i=0; i<4; i++)
			{
				bit = (1 << i);
				if (bit & eflag1)
				{
					print_ecdbg_num((i+8));
					print_ecdbg(" ");
					sysErr.botdrive |= BIT(i+8); //SE_FAIL;
				}
			}
			
			print_ecdbg("\r\n");
			
			if ((eflag1 & 0xF0) != 0)
			{
				print_ecdbg("ERROR on unused channels: PCA9952 - Controller board U8\r\n");
			}
			break;
	}
}

unsigned char numActiveShelves;

unsigned char topEflag0 = 0, topEflag1 = 0, botEflag0 = 0, botEflag1 = 0;

void test_led_driver_channels(void);
void test_led_driver_channels(void)
{
	unsigned char tmp1, tmp2, numShelvesPresent = 0;
	
	
	for (int i=0; i<NUM_SHELVES; i++)
	{
		if (shelf[i].present)
		{
			numShelvesPresent++;
		}
	}
	
	if (numShelvesPresent !=0)
	{
		//Tone down the current so all shelves can be turned on at once
		PCA9952_write_reg(LED_TOP, PCA9952_IREFALL, LED_TEST_DRIVER_CURRENT);
		PCA9952_write_reg(LED_BOTTOM, PCA9952_IREFALL, LED_TEST_DRIVER_CURRENT);

		for (int i=0; i<NUM_SHELVES; i++)
		{
			if (shelf[i].present)
			{
				led_shelf(i, LED_ON);
			}
		}

		PCA9952_write_reg(LED_TOP, PCA9952_MODE2, 0x40); //starts fault test
		PCA9952_write_reg(LED_BOTTOM, PCA9952_MODE2, 0x40); //starts fault test
		
		while (1)
		{
			tmp1 = PCA9952_read_reg(LED_TOP, PCA9952_MODE2);
			
			if ((tmp1 & 0x40) == 0)
			{
				topEflag0 = PCA9952_read_reg(LED_TOP, PCA9952_EFLAG0);
				topEflag1 = PCA9952_read_reg(LED_TOP, PCA9952_EFLAG1);
				
				if ((topEflag0 != 0) || (topEflag1 != 0))
				{
					if (!firstTimeThroughPCA9952)
					{
						print_pca9952_errors(TOP, topEflag0, topEflag1);
					}
				}
				
				break; //fault test for LED_TOP strings is complete
			}
			
		}
		
		while (1)
		{
			tmp2 = PCA9952_read_reg(LED_BOTTOM, PCA9952_MODE2);
			
			if ((tmp2 & 0x40) == 0)
			{
				botEflag0 = PCA9952_read_reg(LED_BOTTOM, PCA9952_EFLAG0);
				botEflag1 = PCA9952_read_reg(LED_BOTTOM, PCA9952_EFLAG1);
				
				if ((botEflag0 != 0) || (botEflag1 != 0))
				{
					if (!firstTimeThroughPCA9952)
					{
						print_pca9952_errors(BOTTOM, botEflag0, botEflag1);
					}
				}
				
				break; //fault test for LED_BOTTOM strings is complete
			}
			
		}
		
		for (int i=0; i<NUM_SHELVES; i++)
		{
			led_shelf(i, LED_OFF);
		}
		
		//Put driver current back to full power
		PCA9952_write_reg(LED_TOP, PCA9952_IREFALL, LED_DRIVER_CURRENT);
		PCA9952_write_reg(LED_BOTTOM, PCA9952_IREFALL, LED_DRIVER_CURRENT);
	}
	
	sysErr.topdrive = topEflag0;
	sysErr.botdrive = (botEflag1 << 8) | botEflag0;
	firstTimeThroughPCA9952 = 0;
}

void set_shelves_active_inactive(void);
void set_shelves_active_inactive(void)
{

	test_led_driver_channels();
	
	numActiveShelves = 0;
	
	for (int i=0; i<NUM_SHELVES; i++)
	{
		shelf[i].active = SHELF_INACTIVE;
		
	}
	
	/* check shelf 0 */
	if (shelf[0].present &&
		shelf[0].devicesPresent && 
		(!ledBrdSide[LED_BRD_0_BOT].maxUsageReached) &&
		(!ledBrdSide[LED_BRD_1_TOP].maxUsageReached) )
	{
		shelf[0].active = SHELF_ACTIVE;
		numActiveShelves++;
		print_ecdbg("Shelf 0 active\r\n");
	}
	
	/* check shelf 1 */
	
	if (shelf[1].present &&
	shelf[1].devicesPresent &&
	(!ledBrdSide[LED_BRD_1_BOT].maxUsageReached) &&
	(!ledBrdSide[LED_BRD_2_TOP].maxUsageReached) )
	{
		shelf[1].active = SHELF_ACTIVE;
		numActiveShelves++;
		print_ecdbg("Shelf 1 active\r\n");
	}
	
	/* check shelf 2 */
	
	if (shelf[2].present &&
	shelf[2].devicesPresent &&
	(!ledBrdSide[LED_BRD_2_BOT].maxUsageReached) &&
	(!ledBrdSide[LED_BRD_3_TOP].maxUsageReached) )
	{
		shelf[2].active = SHELF_ACTIVE;
		numActiveShelves++;
		print_ecdbg("Shelf 2 active\r\n");
	}
	
	/* check shelf 3 */
	
	if (shelf[3].present &&
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
	struct pll_config pcfg;
	
#if 0 //experiment to see if cranking up the clock enables us to read the serial ID chip while running the debugger 16may15
	scif_gclk_opt_t gclkOpt = {SCIF_GCCTRL_RC8M, 0,0};
	
	// Start the 8Mhz Oscillator
	scif_start_rc8M();
	// Set the main clock source as being RC8MHz.
	pm_set_mclk_source(PM_CLK_SRC_RC8M);	

	/* put the clock out on PC19 so we can check to make sure we set it up correctly */
	//Note this code comes from ASF example AVR32 SCIF example 3
	scif_start_gclk(AVR32_SCIF_GCLK_GCLK0PIN, &gclkOpt);
	gpio_enable_module_pin(AVR32_SCIF_GCLK_0_1_PIN, AVR32_SCIF_GCLK_0_1_FUNCTION);

#endif  //experiment to see if cranking up the clock enables us to read the serial ID chip while running the debugger 16may15

//this kinda works for 100MHz, problems with TWIM, but maybe we can work around that 17may15
/*
 * From CLOCK_EXAMPLE31 which changes clock sources on the fly. Trying to get a faster clock so that we can work with the serial ID chip (DS2411) which needs control to 6us. 16may15
 */
	osc_enable(OSC_ID_RC8M);
	pll_config_init(&pcfg, PLL_SRC_RC8M, 1, EC_CPU_CLOCK_100MHZ/OSC_RC8M_NOMINAL_HZ);
	pll_enable(&pcfg, 0);
	sysclk_set_prescalers(1,1,1,1);
	pll_wait_for_lock(0);
	sysclk_set_source(SYSCLK_SRC_PLL0);	


#if 0 //this goes to an exception for some reason 17may15
/*
 * From CLOCK_EXAMPLE31 which changes clock sources on the fly. Trying to get a faster clock so that we can work with the serial ID chip (DS2411) which needs control to 6us. 17may15
 */
	osc_enable(OSC_ID_RC120M);
	osc_wait_ready(OSC_ID_RC120M);
	flash_set_bus_freq(sysclk_get_cpu_hz());
	sysclk_set_source(SYSCLK_SRC_RC120M);
	
#endif
	/* put the clock out on PC19 so we can check to make sure we set it up correctly */
	//Note this code comes from ASF example AVR32 SCIF example 3
//16may15 seems to cause problems, leave out for now	scif_start_gclk(AVR32_SCIF_GCLK_GCLK0PIN, &gclkOpt);
//16may15 seems to cause problems, leave out for now	gpio_enable_module_pin(AVR32_SCIF_GCLK_0_1_PIN, AVR32_SCIF_GCLK_0_1_FUNCTION);

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
	adcifa_configure(adcifa, &adc_config_t, EC_CPU_CLOCK_FREQ);

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

unsigned char calc_sanitize_time(unsigned char shelfIdx);
unsigned char calc_sanitize_time(unsigned char shelfIdx)
{
	unsigned char uSideMinutes, lSideMinutes, minutes, boardIdx, sideIdx;
	
	boardIdx = shelf[shelfIdx].tLedIdx;							//top board in the shelf
	sideIdx = ledBrd[boardIdx].lSideIdx;						//lower side of the top board
	lSideMinutes = ledBrdSide[sideIdx].sanitizeMinutes;
	

	boardIdx = shelf[shelfIdx].bLedIdx;							//bottom board in the shelf					
	sideIdx = ledBrd[boardIdx].uSideIdx;						//upper side of the bottom board
	uSideMinutes = ledBrdSide[sideIdx].sanitizeMinutes;

	minutes = (uSideMinutes >= lSideMinutes) ? uSideMinutes : lSideMinutes; //choose the sanitize time for the more worn-out leds
	
	return (minutes);
	
}


void door_latch_open_kill_all_shelves(void);
void door_latch_open_kill_all_shelves(void)
{
	led_shelf(0, LED_OFF);
	led_shelf(1, LED_OFF);
	led_shelf(2, LED_OFF);
	led_shelf(3, LED_OFF);
	
	print_ecdbg("Door latch opened, kill all shelves for safety.\r\n");
}


/*
 * 2 copies: one each for alternating minutes.
 * Need these areas of flash to erase and be written independently.
 * It's very easy for the power to get shut down during the 
 * one minute updates while the unit is sanitizing. Keeping these
 * 2 areas of flash erasing and updating independently ensures that at
 * least one buffer is intact if the other buffer gets corrupted.
 */


#ifdef SERIAL_ID_AND_ALL_USAGE_COMBINED

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


#else


//! NVRAM data structure located in the flash array.
#if defined (__GNUC__)
__attribute__((__section__(".flash_nvram0")))
#endif
static unsigned char sanitationMinutesFlash[(NUM_SAN_MIN_BUFS_SECTORS * 128)] //TODO: really this should be 58 copies spread out over 2 sectors: how do I do that? Does that matter?
#if defined (__ICCAVR32__)
@ "FLASH_NVRAM0"
#endif
;

#if defined (__GNUC__)
__attribute__((__section__(".flash_nvram1")))
#endif
static unsigned char sanitationCyclesFlash[NUM_SAN_CYCLE_BUFS_SECTORS * 128] //TODO: really this should be 4 copies spread out over 2 sectors: how do I do that? Does it matter?
#if defined (__ICCAVR32__)
@ "FLASH_NVRAM1"
#endif
;

#if defined (__GNUC__)
__attribute__((__section__(".flash_nvram2")))
#endif
static unsigned char usageSerialIdAndUsageHoursFlash[(NUM_USAGE_HOURS_BUFS_SECTORS*128)]
#if defined (__ICCAVR32__)
@ "FLASH_NVRAM2"
#endif
;

#if defined (__GNUC__)
__attribute__((__section__(".flash_nvram3")))
#endif
static unsigned char usageMinutesFlash[NUM_USAGE_MINS_BUFS_SECTORS * 128]
#if defined (__ICCAVR32__)
@ "FLASH_NVRAM3"
#endif
;



#endif //SERIAL_ID_AND_ALL_USAGE_COMBINED

unsigned char minute_count(unsigned char * pMinuteBits)
{
	unsigned char tmpMinBits, minuteCount = 0, bit;
	
	for (int i=0; i<8; i++)
	{
		tmpMinBits = *(pMinuteBits+i);
		bit = 0x80;
		for (int j=0; j<8; j++)
		{
			if ((bit & tmpMinBits) == 0)
			{
				minuteCount++;
			}
			
			bit >>= 1;
		}
	}
	
	return minuteCount;
}

unsigned char inc_minutes(unsigned char * pMinuteBits)
{
	unsigned char bit, invBit, tmpMinBits, minCount = 0;
	
	for (int i=0; i<8; i++)
	{
		bit = 0x80;
		tmpMinBits = (*(pMinuteBits+i));
		
		for (int j=0; j<8; j++)
		{
			minCount++;			
			if (bit & tmpMinBits)
			{
				invBit = (bit ^= 0xFF);
				tmpMinBits &= invBit;
				(*(pMinuteBits+i)) = tmpMinBits;
				return minCount;
			}
			bit >>= 1;
		}
	}
	
	return 0; //we should never get here, but we need this to avoid the warning
}


void reset_minutes(unsigned char * pMinuteBits)
{
	for (int i=0; i<8; i++)
	{
		*(pMinuteBits+i) = 0xFF; //Set to all ones: we will flip one bit from 1 to 0 each minute to save from having to erase flash every minute
	}	
	
}

#define STRINGS_MATCH 0

#ifdef SERIAL_ID_AND_ALL_USAGE_COMBINED

unsigned char usage_idx(unsigned char sel, unsigned char * idPtr, unsigned char top_botn);
unsigned char usage_idx(unsigned char sel, unsigned char * idPtr, unsigned char top_botn)

#else

unsigned char usage_idx(unsigned char * idPtr, unsigned char top_botn);
unsigned char usage_idx(unsigned char * idPtr, unsigned char top_botn)

#endif //SERIAL_ID_AND_ALL_USAGE_COMBINED
{
	unsigned char tmpBoardId[6];
	
	for (unsigned char i=0; i<(NUM_SETS_LED_BOARD_SIDES * NUM_LED_BOARD_SIDES); i++)
	{
		
#if 0 //This just isn't working the way it should, maybe trying to do too much in one line for this compiler		
		if ((strncmp((char*)idPtr, (char*)(&usageShdw[sel].u[i].id[0]),6)) && (usageShdw[sel].u[i].top_botn == top_botn) == STRINGS_MATCH)
		{
			return (i); //Found a match!
		}
#endif

#ifdef SERIAL_ID_AND_ALL_USAGE_COMBINED
		if (usageShdw[sel].u[i].slotFilled)
		{
			tmpBoardId[0] = *(idPtr+0);
			tmpBoardId[1] = *(idPtr+1);
			tmpBoardId[2] = *(idPtr+2);
			tmpBoardId[3] = *(idPtr+3);
			tmpBoardId[4] = *(idPtr+4);
			tmpBoardId[5] = *(idPtr+5);
			
		
			if (tmpBoardId[0] == usageShdw[sel].u[i].id[0]) {
				if (tmpBoardId[1] == usageShdw[sel].u[i].id[1]) {
					if (tmpBoardId[2] == usageShdw[sel].u[i].id[2]) {
						if (tmpBoardId[3] == usageShdw[sel].u[i].id[3]) {
							if (tmpBoardId[4] == usageShdw[sel].u[i].id[4]) {
								if (tmpBoardId[5] == usageShdw[sel].u[i].id[5]) {
									if (top_botn == usageShdw[sel].u[i].top_botn)
									{
										return (i); //found a match!

									} //check top_botn match
								} //tmpBoardId[5]
							} //tmpBoardId[4]
						} //tmpBoardId[3]
					} //tmpBoardId[2]
				} //tmpBoardId[1]
			} //tmpBoardId[0]
		} //if slotFilled (don't check against slots that haven't been assigned
	} //for each slot in usageShdw[sel]
	
#else

		if (ush.u[i].slotFilled)
		{
			tmpBoardId[0] = *(idPtr+0);
			tmpBoardId[1] = *(idPtr+1);
			tmpBoardId[2] = *(idPtr+2);
			tmpBoardId[3] = *(idPtr+3);
			tmpBoardId[4] = *(idPtr+4);
			tmpBoardId[5] = *(idPtr+5);
			
			
			if (tmpBoardId[0] == ush.u[i].id[0]) {
				if (tmpBoardId[1] == ush.u[i].id[1]) {
					if (tmpBoardId[2] == ush.u[i].id[2]) {
						if (tmpBoardId[3] == ush.u[i].id[3]) {
							if (tmpBoardId[4] == ush.u[i].id[4]) {
								if (tmpBoardId[5] == ush.u[i].id[5]) {
									if (top_botn == ush.u[i].top_botn)
									{
										return (i); //found a match!

									} //check top_botn match
								} //tmpBoardId[5]
							} //tmpBoardId[4]
						} //tmpBoardId[3]
					} //tmpBoardId[2]
				} //tmpBoardId[1]
			} //tmpBoardId[0]
		} //if slotFilled (don't check against slots that haven't been assigned
	} //for each slot in ush
	
#endif //SERIAL_ID_AND_ALL_USAGE_COMBINED
	
	return NO_LED_BOARD_PRESENT; //no match found
}

#if 0 //SERIAL_ID_AND_ALL_USAGE_COMBINED
void load_usage_indeces(unsigned char sel)
{
	unsigned char top_botn, brdIdx;
		
	for (int i=0; i<NUM_LED_BOARD_SIDES; i++)
	{
		brdIdx = ledBrdSide[i].boardIdx;
		
		if (ledBrd[brdIdx].present)
		{
			top_botn = i%2;
			
			usageIdx[sel][i] = usage_idx(sel, &ledBrd[brdIdx].id[0], top_botn); //TODO: should change this nomenclature to upper/lower, we are talking about board sides here, not which board in the shelf, be consistent
		}
		else
		{
			usageIdx[sel][i] = NO_LED_BOARD_PRESENT;
		}
	}
}
#endif //SERIAL_ID_AND_ALL_USAGE_COMBINED

void load_usage_indeces(void)
{
	unsigned char top_botn, brdIdx;
	
	for (int i=0; i<NUM_LED_BOARD_SIDES; i++)
	{
		brdIdx = ledBrdSide[i].boardIdx;
		
		if (ledBrd[brdIdx].present)
		{
			top_botn = i%2;
			
			usageIdx[i] = usage_idx(&ledBrd[brdIdx].id[0], top_botn); //TODO: should change this nomenclature to upper/lower, we are talking about board sides here, not which board in the shelf, be consistent
		}
		else
		{
			usageIdx[i] = NO_LED_BOARD_PRESENT;
		}
	}
}


enum{CHECKSUM_INVALID, CHECKSUM_VALID};

#if 0 //SERIAL_ID_AND_ALL_USAGE_COMBINED
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

#endif //SERIAL_ID_AND_ALL_USAGE_COMBINED

enum {SUCCESS, ERROR};


#ifdef SERIAL_ID_AND_ALL_USAGE_COMBINED
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

#else
unsigned char test_flash(unsigned char sel)
{
	volatile void* memPtr;
	unsigned char pattern[4] = {0xFF, 0x00, 0xAA, 0x55}, ubyte;
	unsigned char *ubPtr;
	unsigned long memSize;
	
	switch(sel)
	{
		case 0:
			memPtr = &sanitationMinutesFlash;
			memSize = NUM_SAN_MIN_BUFS_SECTORS * 128;
			break;
		case 1:
			memPtr = &sanitationCyclesFlash;
			memSize = NUM_SAN_CYCLE_BUFS_SECTORS * 128;
			break;
		case 2:
			memPtr = &usageSerialIdAndUsageHoursFlash;
			memSize = NUM_USAGE_HOURS_BUFS_SECTORS * 128;
			break;
		case 3:
			memPtr = &usageMinutesFlash;
			memSize = NUM_USAGE_MINS_BUFS_SECTORS * 128;
			break;
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


#endif //SERIAL_ID_AND_ALL_USAGE_COMBINED

#ifndef SERIAL_ID_AND_ALL_USAGE_COMBINED

unsigned char calc_region_checksum(unsigned char sel);
unsigned char calc_region_checksum(unsigned char sel)
{
	unsigned char csum;

	switch(sel)
	{
		case 0: //san minutes
			csum = ((sanm.mins ^ 0xFF) & 0xFF);
			break;
		case 1: //san cycles
			csum = ((sanc.cycles ^ 0xFF) & 0x1F);
			break;
		case 2: //usage hours
			csum = 0;
			for (int i=0; i<(NUM_SETS_LED_BOARD_SIDES * NUM_LED_BOARD_SIDES); i++)
			{
				csum += ush.u[i].hours;
				csum += ush.u[i].id[0];
				csum += ush.u[i].id[1];
				csum += ush.u[i].id[2];
				csum += ush.u[i].id[3];
				csum += ush.u[i].id[4];
				csum += ush.u[i].id[5];
				csum += ush.u[i].maxUsageReached;
				csum += ush.u[i].slotFilled;
				csum += ush.u[i].top_botn;
			}
			csum = ((csum ^ 0xFF) & 0xFF);
			break;
		case 3: //usage mins
			csum = 0;
			for (int i=0; i<(NUM_SETS_LED_BOARD_SIDES * NUM_LED_BOARD_SIDES); i++)
			{
				csum += um.mins[i];
			}
			csum = ((csum ^ 0xFF) & 0xFF);
			break;
	}
	
	return csum;	
}

unsigned char eval_region(unsigned char sel);
unsigned char eval_region(unsigned char sel)
{
	CHASSIS_SANITATION_MINUTES		tmpSanm;
	CHASSIS_SANITATION_CYCLES		tmpSanc;
	USAGE_SERIAL_ID_AND_HOURS_SET	tmpUsh;
	USAGE_MINS_SET					tmpUm;
	
	unsigned char					csum;
	long							flashOffset;
	long							tmpFlashOffset;
	unsigned char					retVal = ERROR;
	

	unsigned long tmpHours, uHours, tmpMinutes, uMinutes;
	
	switch (sel)
	{
		case 0: //san minutes
			
			memset(&tmpSanm, 0x00, sizeof(sanm));
		
			for (unsigned int i=0; i<(NUM_SAN_MIN_BUFS_PER_SECTOR * NUM_SAN_MIN_BUFS_SECTORS); i++)
			{
				if (i<NUM_SAN_MIN_BUFS_PER_SECTOR)
				{
					flashOffset =  (i*sizeof(sanm));
				}
				else
				{
					flashOffset = (128 + ((i - NUM_SAN_MIN_BUFS_PER_SECTOR) * sizeof(sanm)));
				}

				tmpFlashOffset = flashOffset + (unsigned long)sanitationMinutesFlash;
				memcpy(&sanm, (const void*) tmpFlashOffset, sizeof(sanm));
				
				csum = calc_region_checksum(0);
				
				if (csum == sanm.csum) //checksum is good
				{
					retVal = SUCCESS; //we have at least one good copy
					
					if (sanm.mins > tmpSanm.mins)
					{
						memcpy(&tmpSanm, &sanm, sizeof(sanm));
						sanMinFlashIdx = i; //this is the new best copy	
					}
				}
			}
			break;
		case 1: //san cycles

			memset(&tmpSanc, 0x00, sizeof(sanc));

			for (unsigned int i=0; i<(NUM_SAN_CYCLE_BUFS_PER_SECTOR * NUM_SAN_CYCLE_BUFS_SECTORS); i++)
			{
				if (i<NUM_SAN_CYCLE_BUFS_PER_SECTOR)
				{
					flashOffset = (i*sizeof(sanc));
				}
				else
				{
					flashOffset = (128 + ((i - NUM_SAN_CYCLE_BUFS_PER_SECTOR) * sizeof(sanc)));
				}
				
				tmpFlashOffset = flashOffset + (unsigned long) sanitationCyclesFlash+flashOffset;
				memcpy(&sanc, (const void*) tmpFlashOffset, sizeof(sanc));
				
				csum = calc_region_checksum(1);
				
				if (csum == sanc.csum) //checksum is good
				{
					retVal = SUCCESS; //we have at least one good copy
					
					if (sanc.cycles > tmpSanc.cycles)
					{
						memcpy(&tmpSanc, &sanc, sizeof(sanc));
						sanCycleFlashIdx = i; //this is the new best copy
					}
				}
			}
			break;
		case 2: //usage hours
			
			memset(&tmpUsh, 0x00, sizeof(ush));

			for (unsigned int i=0; i<(NUM_USAGE_HOURS_BUFS_SECTORS / NUM_USAGE_HOURS_SECTORS_PER_BUF); i++)
			{
				flashOffset = (i * NUM_USAGE_HOURS_SECTORS_PER_BUF * 128);
				
				tmpFlashOffset = flashOffset + (unsigned long) usageSerialIdAndUsageHoursFlash;
				
				memcpy(&ush, (const void*) tmpFlashOffset, sizeof(ush));
				
				csum = calc_region_checksum(2);
				
				if (csum == ush.csum) //checksum is good
				{
					retVal = SUCCESS; //we have at least one good copy
					
					tmpHours = 0;
					uHours = 0;
					
					for (int i=0; i<(NUM_SETS_LED_BOARD_SIDES * NUM_LED_BOARD_SIDES); i++)
					{
						if (tmpUsh.u[i].slotFilled)
						{	
							tmpHours += tmpUsh.u[i].hours;
						}
						
						if (ush.u[i].slotFilled)
						{
							uHours += ush.u[i].hours;
						}
					}
					
					if (uHours > tmpHours)
					{
						memcpy(&tmpUsh, &ush, sizeof(ush));
						ushFlashIdx = i; //this is the new best copy
					}
				}
			}
			break;
		case 3: //usage minutes

			memset(&tmpUm, 0x00, sizeof(um));
			
			for (unsigned int i=0; i<(NUM_USAGE_MINS_BUFS_PER_SECTOR * NUM_USAGE_MINS_BUFS_SECTORS); i++)
			{
				flashOffset = (i * NUM_USAGE_HOURS_SECTORS_PER_BUF * 128);
				
				tmpFlashOffset = flashOffset + (unsigned long) usageMinutesFlash;
				
				memcpy(&um, (const void*) tmpFlashOffset, sizeof(um));
				
				csum = calc_region_checksum(3);
				
				if (csum == um.csum) //checksum is good
				{
					retVal = SUCCESS; //we have at least one good copy
					
					tmpMinutes = 0;
					uMinutes = 0;
					
					for (int i=0; i<(NUM_SETS_LED_BOARD_SIDES * NUM_LED_BOARD_SIDES); i++)
					{
						//TODO: I should be checking the ush struct to see if the slot is filled, but i don't have a good way of syncing ush and um right now. In the meantime, just make sure that um.mins[i] is 0 if not used.
						tmpMinutes += tmpUm.mins[i];
						uMinutes += um.mins[i];
					}
					
					if (uMinutes > tmpMinutes)
					{
						memcpy(&tmpUm, &um, sizeof(um));
						umFlashIdx = i; //this is the new best copy
					}
				}
			}
			break;
	}
	
	return retVal;
}

unsigned char write_region_to_flash(unsigned char sel, unsigned char idx, unsigned char csum);
unsigned char write_region_to_flash(unsigned char sel, unsigned char idx, unsigned char csum)
{
	unsigned long tmpFlashOffset, flashOffset;
	bool eraseFlag;
	unsigned char tmpIdx;
	
	if (idx == 0xFF) //use the default system index
	{
		switch(sel)
		{
			case 0:
				tmpIdx = sanMinFlashIdx;
				break;
			case 1:
				tmpIdx = sanCycleFlashIdx;
				break;
			case 2:
				tmpIdx = ushFlashIdx;
				break;
			case 3:
				tmpIdx = umFlashIdx;
				break;
		}
	}
	else //use the specific index passed to this function
	{
		tmpIdx = idx;
	}
	
	switch (sel)
	{
		case 0: //san minutes
			//NOTE: this is not as parameterized as it should be, only good for 2 sectors, but good enough for now. 
			
			sanm.csum = csum;
			
			if (tmpIdx < NUM_SAN_MIN_BUFS_PER_SECTOR)
			{
				flashOffset = tmpIdx * sizeof(sanm);
			}
			else
			{
				flashOffset = 128 + ((tmpIdx - NUM_SAN_MIN_BUFS_PER_SECTOR) * sizeof(sanm));
			}
			
			if ((tmpIdx == 0) || (tmpIdx == NUM_SAN_MIN_BUFS_PER_SECTOR))
			{
				eraseFlag = true;
			}
			else
			{
				eraseFlag = false; //only erase the sector when we are writing the first entry in the sector
			}
			
			tmpFlashOffset = flashOffset + (unsigned long) sanitationMinutesFlash;
			
			flashc_memcpy((volatile void*)tmpFlashOffset, &sanm, sizeof(sanm), eraseFlag);
			
			break;

		case 1: //san cycles
			//NOTE: this is not as parameterized as it should be, only good for 2 sectors, but good enough for now.
			
			sanc.csum = csum;
			
			if (tmpIdx < NUM_SAN_CYCLE_BUFS_PER_SECTOR)
			{
				flashOffset = tmpIdx * sizeof(sanc);
			}
			else
			{
				flashOffset = 128 + ((tmpIdx - NUM_SAN_CYCLE_BUFS_PER_SECTOR) * sizeof(sanc));
			}
			
			if ((tmpIdx == 0) || (tmpIdx == NUM_SAN_CYCLE_BUFS_PER_SECTOR))
			{
				eraseFlag = true;
			}
			else
			{
				eraseFlag = false; //only erase the sector when we are writing the first entry in the sector
			}
			
			tmpFlashOffset = flashOffset + (unsigned long) sanitationCyclesFlash;
			
			flashc_memcpy((volatile void*)tmpFlashOffset, &sanc, sizeof(sanc), eraseFlag);
			
			break;

		case 2: //usage hours
			ush.csum = csum;

			flashOffset = tmpIdx * NUM_USAGE_HOURS_SECTORS_PER_BUF * 128;
			
			tmpFlashOffset = flashOffset + (unsigned long) usageSerialIdAndUsageHoursFlash;
						
			flashc_memcpy((volatile void*)tmpFlashOffset, &ush, sizeof(ush), true); //we erase every time because this structure takes up multiple sectors
			break;
		case 3: //usage minutes
			//NOTE: this is not as parameterized as it should be, but good enough for now.
			um.csum = csum;
			
			flashOffset = tmpIdx * 128;
			
			tmpFlashOffset = flashOffset + (unsigned long) usageMinutesFlash;

			flashc_memcpy((volatile void*)tmpFlashOffset, &um, sizeof(um), true); //we erase every time because this structure takes up one whole sector
			break;
	}
	
	return SUCCESS;	
}

void copy_region_to_another_sector(unsigned char sel);
void copy_region_to_another_sector(unsigned char sel)
{
	unsigned char tmpIdx, csum;
	
	switch (sel)
	{
		case 0: //san minutes
			if (sanMinFlashIdx < NUM_SAN_MIN_BUFS_PER_SECTOR)
			{
				tmpIdx = sanMinFlashIdx + NUM_SAN_MIN_BUFS_PER_SECTOR;
			}
			else
			{
				tmpIdx = sanMinFlashIdx - NUM_SAN_MIN_BUFS_PER_SECTOR;
			}
			
			csum = calc_region_checksum(0);
			write_region_to_flash(0, tmpIdx, csum);
			break;
		case 1: //san cycles
			if (sanCycleFlashIdx < NUM_SAN_CYCLE_BUFS_PER_SECTOR)
			{
				tmpIdx = sanCycleFlashIdx + NUM_SAN_CYCLE_BUFS_PER_SECTOR;
			}
			else
			{
				tmpIdx = sanCycleFlashIdx - NUM_SAN_CYCLE_BUFS_PER_SECTOR;
			}
			csum = calc_region_checksum(1);
			write_region_to_flash(1, tmpIdx, csum);
			break;
		case 2: //usage hours
			tmpIdx = ((ushFlashIdx + 1) & 1);
			csum = calc_region_checksum(2);
			write_region_to_flash(2, tmpIdx, csum);
			break;
		case 3: //usage minutes
			tmpIdx = umFlashIdx + (NUM_USAGE_MINS_BUFS_SECTORS/2);
			if (tmpIdx > NUM_USAGE_MINS_BUFS_SECTORS)
			{
				tmpIdx -= NUM_USAGE_MINS_BUFS_SECTORS; //wrap if necessary
			}
			csum = calc_region_checksum(3);
			write_region_to_flash(3, tmpIdx, csum);
			break;
	}
	
}

#endif //SERIAL_ID_AND_ALL_USAGE_COMBINED


#define USAGE_FULL 0xFF

#ifdef SERIAL_ID_AND_ALL_USAGE_COMBINED
unsigned char find_first_open_usage_slot(unsigned char sel);
unsigned char find_first_open_usage_slot(unsigned char sel)
#else
unsigned char find_first_open_usage_slot(void);
unsigned char find_first_open_usage_slot(void)
#endif
{
	for (unsigned int i=0; i<(NUM_SETS_LED_BOARD_SIDES * NUM_LED_BOARD_SIDES); i++)
	{
		
#ifdef SERIAL_ID_AND_ALL_USAGE_COMBINED		
		if (!usageShdw[sel].u[i].slotFilled)
#else
		if (!ush.u[i].slotFilled)
#endif //SERIAL_ID_AND_ALL_USAGE_COMBINED
		{
			return i;
		}
	}
	
	print_ecdbg("No more room for LED board info. Cannot track minute usage for additional boards.\r\n");
	
	sysErr.usageStructsFull = FAIL;
	
	return USAGE_FULL; //Error, no open slots
}

#if 0 //SERIAL_ID_AND_ALL_USAGE_COMBINED
void add_new_led_board_sides_to_usage(unsigned char sel)
#endif
void add_new_led_board_sides_to_usage(void)

{
	unsigned char firstOpenSlot, slotAssignment, brdIdx, top_botn;
	
	//NOTE that load_usage_indeces() must have been run already for this function to work. 
	// i.e., usageIdx[][] must be populated.
	
#ifdef SERIAL_ID_AND_ALL_USAGE_COMBINED
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
#else

	firstOpenSlot = find_first_open_usage_slot();
	
	slotAssignment = firstOpenSlot;
	
	for (unsigned char i=0; i<NUM_LED_BOARD_SIDES; i++)
	{
		brdIdx = ledBrdSide[i].boardIdx;
		
		top_botn = (i%2) ? TOP : BOTTOM; //odd sides are top, even sides are bottom
		
		if ((ledBrd[brdIdx].present) && (usageIdx[i] == NO_LED_BOARD_PRESENT)) //TODO: do I need the NO_LED_BOARD_PRESENT check? this should always be open at this point
		{
			strncpy((char*)&ush.u[slotAssignment].id[0], (char*)&ledBrd[brdIdx].id[0],6);
			
			ush.u[slotAssignment].top_botn = top_botn;
			
			ush.u[slotAssignment].slotFilled = 1;

			usageIdx[i] = slotAssignment++;

#endif //SERIAL_ID_AND_ALL_USAGE_COMBINED			
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
		
#ifdef SERIAL_ID_AND_ALL_USAGE_COMBINED		
		csum += usageShdw[sel].u[i].minutes;
		
// We are not including minuteBits in the structure checksum on purpose since we are updating minuteBits every minute, but only writing the whole structure to flash once per hour
//		for (int j=0; j<8; j++)
//		{
//			csum += usageShdw[sel].u[i].minuteBits[j];
//		}
		
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
	
	csum += usageShdw[sel].totalSanitationCycles;
	csum += usageShdw[sel].totalSanitationMinutes;
#else
		csum += ush.u[i].id[0];
		csum += ush.u[i].id[1];
		csum += ush.u[i].id[2];
		csum += ush.u[i].id[3];
		csum += ush.u[i].id[4];
		csum += ush.u[i].id[5];
		
		csum += ush.u[i].maxUsageReached;
		csum += ush.u[i].top_botn;
		csum += ush.u[i].slotFilled;
		csum += ush.u[i].hours;

	}
	
#endif //SERIAL_ID_AND_ALL_USAGE_COMBINED	
	return csum;
}

#if 0 //SERIAL_ID_AND_ALL_USAGE_COMBINED
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
		flashc_memcpy(serialIdAndUsageFlashOne, &usageShdw[1], sizeof(usageShdw[1]),true);
	}
}
#endif //SERIAL_ID_AND_ALL_USAGE_COMBINED

#ifdef SERIAL_ID_AND_ALL_USAGE_COMBINED
//TODO: this doesn't seem to work. Should we fix it?
void write_usage_to_flash_no_erase(unsigned char sel);
void write_usage_to_flash_no_erase(unsigned char sel)
{
	if (sel == 0)
	{
		flashc_memcpy(serialIdAndUsageFlashZero, &usageShdw[0], sizeof(usageShdw[0]),false);
	}
	else
	{
		flashc_memcpy(serialIdAndUsageFlashOne, &usageShdw[1], sizeof(usageShdw[1]),false);
	}
}

#endif //SERIAL_ID_AND_ALL_USAGE_COMBINED


unsigned long calc_usage_current_led_boards(unsigned char sel);
unsigned long calc_usage_current_led_boards(unsigned char sel)
{
	unsigned long minutes = 0; 
	unsigned char idx;
	
	for (unsigned char i=0; i<NUM_LED_BOARD_SIDES; i++)
	{
#ifdef SERIAL_ID_AND_ALL_USAGE_COMBINED
		if (usageIdx[sel][i] != NO_LED_BOARD_PRESENT)
		{
			idx = usageIdx[sel][i];
			
			minutes += usageShdw[sel].u[idx].minutes;
			
#else
		if (usageIdx[i] != NO_LED_BOARD_PRESENT)
		{
			idx = usageIdx[i];

			minutes += um.mins[idx];

#endif //SERIAL_ID_AND_ALL_USAGE_COMBINED			
			
//TODO: come back to this if we fix the flash with no erase problem			minutes += minute_count(&usageShdw[sel].u[idx].minuteBits[0]);
		}
	}
	
	return minutes;
}

void increment_ledBoard_usage_min(void);

#ifdef SERIAL_ID_AND_ALL_USAGE_COMBINED
void increment_ledBoard_usage_min(void)
{
	SERIAL_ID_AND_USAGE *tmp;
	unsigned char topLEDboardLowerSideIdx;
	unsigned char bottomLEDboardUpperSideIdx;
	unsigned char topUIdx;
	unsigned char bottomUIdx;
	unsigned char hourRollover = 0;
	
	usageShdw[0].totalSanitationMinutes++;
	usageShdw[1].totalSanitationMinutes++;


	for (unsigned char i=0; i<NUM_SHELVES; i++) //check every active shelf
	{
		if (shelf[i].active == SHELF_ACTIVE)
		{
			topLEDboardLowerSideIdx = ledBrd[shelf[i].tLedIdx].lSideIdx;
			bottomLEDboardUpperSideIdx = ledBrd[shelf[i].bLedIdx].uSideIdx;
		
			topUIdx = ledBrdSide[topLEDboardLowerSideIdx].ushdwIdx;
			bottomUIdx = ledBrdSide[bottomLEDboardUpperSideIdx].ushdwIdx;
		
			for (unsigned char j=0; j<2; j++) //for each copy of usageShdw[] (update both copies every time even though we only write one to flash each time)
			{
				for (unsigned char k=0; k<2; k++) //for each board side in the shelf
				{
					switch (k)
					{
						case 0:
							tmp = &usageShdw[j].u[topUIdx];
							break;
						case 1:
							tmp = &usageShdw[j].u[bottomUIdx];
							break;
					}

					tmp->minutes = tmp->minutes + 1;
					if ((tmp->minutes %60) == 0)
					{
//TODO: come back to this if we fix the flash no erase problem						reset_minutes(tmp->minuteBits);

						if (j == hourPingPong)
						{
							hourRollover++; //count number of board sides that had hours rollover this pass for the current hourPingPong selection
						}
					
						if (tmp->minutes >= (2000 * 60)) //2000 hours * 60 minutes per hour
						{
							tmp->maxUsageReached = 1; //And...we're done. Reached 2000 hours.
						}
					}//if ((tmp->minutes %60) == 0)
				} //for each board side in the shelf (k)
			} //for each copy of usageShdw
		} //if (shelf[i].active)
	} //for (i=0; i<NUM_SHELVES; i++)
	
	if (hourRollover)
	{
		usageShdw[hourPingPong].csum = calc_usage_csum(hourPingPong);
		write_usage_to_flash(hourPingPong);
		hourRollover = 0; //reset for next pass
		hourPingPong++;
		hourPingPong &= 1; //toggle between 0 (EVEN) and 1 (ODD)
	}
	else
	{
//17may15 TODO: this doesn't seem to work, maybe we can live without it for now.		write_usage_to_flash_no_erase(minPingPong); //all data should be the same except minuteBits, don't update the checksum when we are only updating minuteBits
	}
	
	minPingPong++;
	minPingPong &= 1; //toggle between 0 (EVEN) and 1 (ODD)
}

#else

void inc_sanMins(void);
void inc_sanMins(void)
{
	sanm.csum = calc_region_checksum(0);
	write_region_to_flash(0, 0xFF, sanm.csum);
	if (++sanMinFlashIdx > (NUM_SAN_MIN_BUFS_PER_SECTOR * NUM_SAN_MIN_BUFS_SECTORS))
	{
		sanMinFlashIdx = 0;
	}
}

void inc_sanCycles(void);
void inc_sanCycles(void)
{
	sanc.csum = calc_region_checksum(1);
	write_region_to_flash(1, 0xFF, sanc.csum);
	if (++sanCycleFlashIdx > (NUM_SAN_CYCLE_BUFS_PER_SECTOR * NUM_SAN_CYCLE_BUFS_SECTORS))
	{
		sanCycleFlashIdx = 0;
	}
}


void increment_ledBoard_usage_min(void)
{
	unsigned char idx;
	unsigned char topLEDboardLowerSideIdx;
	unsigned char bottomLEDboardUpperSideIdx;
	unsigned char topUIdx;
	unsigned char bottomUIdx;
	unsigned char hourRollover = 0;
	
	inc_sanMins();

	for (unsigned char i=0; i<NUM_SHELVES; i++) //check every active shelf
	{
		if (shelf[i].active == SHELF_ACTIVE)
		{
			topLEDboardLowerSideIdx = ledBrd[shelf[i].tLedIdx].lSideIdx;
			bottomLEDboardUpperSideIdx = ledBrd[shelf[i].bLedIdx].uSideIdx;
			
			topUIdx = ledBrdSide[topLEDboardLowerSideIdx].ushdwIdx;
			bottomUIdx = ledBrdSide[bottomLEDboardUpperSideIdx].ushdwIdx;
			
			for (unsigned char k=0; k<2; k++) //for each board side in the shelf
			{
				switch (k)
				{
					case 0:
						idx = topUIdx;
						break;
					case 1:
						idx = bottomUIdx;
						break;

				}

				um.mins[idx] = um.mins[idx] + 1;
				if (((um.mins[idx]) %60) == 0)
				{
					hourRollover++; //count number of board sides that had hours rollover this pass for the current hourPingPong selection
					ush.u[idx].hours = ush.u[idx].hours + 1;
						
					if ((ush.u[idx].hours) >= 2000) //2000 hours * 60 minutes per hour
					{
						ush.u[idx].maxUsageReached = 1; //And...we're done. Reached 2000 hours.
					}
				}//if ((minutes %60) == 0)
			} //for each board side in the shelf (k)
		} //if (shelf[i].active)
	} //for (i=0; i<NUM_SHELVES; i++)
	

	um.csum = calc_region_checksum(3);
	write_region_to_flash(3, 0xFF, um.csum);
	if (++umFlashIdx > NUM_USAGE_MINS_BUFS_SECTORS)
	{
		umFlashIdx = 0;
	}

	if (hourRollover)
	{
		ush.csum = calc_region_checksum(2);
		write_region_to_flash(2, 0xFF, ush.csum);
		if (++ushFlashIdx > (NUM_USAGE_HOURS_BUFS_SECTORS/NUM_USAGE_HOURS_SECTORS_PER_BUF))
		{
			ushFlashIdx = 0;
		}

		hourRollover = 0; //reset for next pass
	}
}



#endif //SERIAL_ID_AND_ALL_USAGE_COMBINED


void init_shelf_n_ledBrd_structs(void);
void init_shelf_n_ledBrd_structs(void)
{
	
	for (int i=0; i<NUM_SHELVES; i++)
	{
		shelf[i].present = 0;
		shelf[i].devicesPresent = 0;
		shelf[i].active = 0;
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
	
#if SERIAL_ID_AND_ALL_USAGE_COMBINED
	for (int i=0; i<2; i++)
	{
		for (int j=0; j<NUM_LED_BOARD_SIDES; j++)
		{
			usageIdx[i][j] = NO_LED_BOARD_PRESENT;
		}
	}
#else
	for (int j=0; j<NUM_LED_BOARD_SIDES; j++)
	{
		usageIdx[j] = NO_LED_BOARD_PRESENT;
	}
#endif

}

#if SERIAL_ID_AND_ALL_USAGE_COMBINED
void load_usageIdx_to_ledBrdSide(unsigned char sel);
void load_usageIdx_to_ledBrdSide(unsigned char sel)
{
	for (int i=0; i<NUM_LED_BOARD_SIDES; i++)
	{
		ledBrdSide[i].ushdwIdx = usageIdx[sel][i];
	}
}

#else

void load_usageIdx_to_ledBrdSide(void);
void load_usageIdx_to_ledBrdSide(void)
{
	for (int i=0; i<NUM_LED_BOARD_SIDES; i++)
	{
		ledBrdSide[i].ushdwIdx = usageIdx[i];
	}
}

#endif

#if 0 //SERIAL_ID_AND_ALL_USAGE_COMBINED
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
		print_ecdbg("Flash usage area 0 good.\r\n");
		load_usage_indeces(0);
	}
	else
	{
		memset(&usageShdw[0], 0x00, sizeof(usageShdw[0]));
		
		for (int i=0; i<(NUM_LED_BOARD_SIDES * NUM_SETS_LED_BOARD_SIDES); i++)
		{
			reset_minutes(&usageShdw[0].u[i].minuteBits[0]);
		}
	}
	
	if (usage1good)
	{
		print_ecdbg("Flash usage area 1 good.\r\n");
		load_usage_indeces(1);
	}
	else
	{
		memset(&usageShdw[1], 0x00, sizeof(usageShdw[1]));

		for (int i=0; i<(NUM_LED_BOARD_SIDES * NUM_SETS_LED_BOARD_SIDES); i++)
		{
			reset_minutes(&usageShdw[1].u[i].minuteBits[0]);
		}
	}
	
	if ((!usage0good) && (!usage1good)) //Chassis is probably powering up for the first time
	{
		if (test_flash(0) == ERROR)
		{
			print_ecdbg("Flash usage area 0 ERROR.\r\n");
			sysErr.flashArea |= BIT(0); //SE_FAIL;
			chassis_error();
		}
		if (test_flash(1) == ERROR)
		{
			print_ecdbg("Flash usage area 1 ERROR.\r\n");
			sysErr.flashArea|= BIT(1); //SE_FAIL;
			chassis_error();
		}
		add_new_led_board_sides_to_usage(0);
		load_usageIdx_to_ledBrdSide(0);
		usageShdw[0].csum = calc_usage_csum(0);
		copy_usage_to_usage(1,0);
		write_usage_to_flash(0);
		write_usage_to_flash(1);
		minPingPong = 0;
		hourPingPong = 0;
		
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
			print_ecdbg("Flash usage area ");
			print_ecdbg_num(older);
			print_ecdbg(" ERROR.\r\n");
			sysErr.flashArea|= BIT(older); //SE_FAIL;
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
			print_ecdbg("Flash usage area ");
			print_ecdbg_num(newer);
			print_ecdbg(" ERROR.\r\n");
			sysErr.flashArea |= BIT(newer); //SE_FAIL;
			chassis_error();
		}

		write_usage_to_flash(newer);
		minPingPong = 0;
		hourPingPong = 0;
		
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
			print_ecdbg("Flash usage area ");
			print_ecdbg_num(bad);
			print_ecdbg(" ERROR.\r\n");
			sysErr.flashArea |= BIT(bad); //SE_FAIL;
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
			print_ecdbg("Flash usage area ");
			print_ecdbg_num(good);
			print_ecdbg(" ERROR.\r\n");
			sysErr.flashArea |= BIT(good); //SE_FAIL;
			chassis_error();
		}
		
		write_usage_to_flash(good);
		minPingPong = 0;
		hourPingPong = 0;
	}
	
}

#endif //SERIAL_ID_AND_ALL_USAGE_COMBINED

void init_led_board_info(void);
void init_led_board_info(void)
{
	unsigned char regionGood[4];
	unsigned char csum;
	
	init_shelf_n_ledBrd_structs();
	
	read_led_board_serial_ids();
	
	for (int i=0; i<4; i++)
	{
		regionGood[i] = eval_region(i);
	}
	
	if (regionGood[0] && regionGood[1] && regionGood[2] && regionGood[3])
	{
		print_ecdbg("All 4 flash regions have good data sets.\r\n");

		load_usage_indeces();
		
		add_new_led_board_sides_to_usage();
		load_usageIdx_to_ledBrdSide();

		//san minutes
		csum = calc_region_checksum(0);
		write_region_to_flash(0, 0xFF, csum);
		copy_region_to_another_sector(0);

		//san cycles
		csum = calc_region_checksum(1);
		write_region_to_flash(1,  0xFF, csum);
		copy_region_to_another_sector(1);

		//usage hours
		csum = calc_region_checksum(2);
		write_region_to_flash(2,  0xFF, csum);
		copy_region_to_another_sector(2);

		//usage minutes
		csum = calc_region_checksum(3);
		write_region_to_flash(3,  0xFF, csum);
		copy_region_to_another_sector(3);
	}
	else
	{
		memset(&ush, 0x00, sizeof(ush));	//serial id's and usage hours
		memset(&um, 0x00, sizeof(um));		//usage minutes
		memset(&sanm, 0x00, sizeof(sanm));	//total chassis sanitation minutes
		memset(&sanc, 0x00, sizeof(sanc));	//total chassis sanitation hours

		for (int i=0; i<4; i++)
		{
			if (test_flash(i) == ERROR)
			{
				print_ecdbg("Flash area ERROR: region ");
				print_ecdbg_num(i);
				print_ecdbg("\r\n");
				sysErr.flashArea |= BIT(0); //SE_FAIL;
				chassis_error();
			}

		}
		add_new_led_board_sides_to_usage();
		load_usageIdx_to_ledBrdSide();

		for (int i=0; i<4; i++)
		{
			unsigned char csum;
			csum = calc_region_checksum(i);
			write_region_to_flash(i,  0xFF, csum);
			copy_region_to_another_sector(i);
		}
	}
}

void show_chassis_status_info(void);
void show_chassis_status_info(void)
{
	char pStr[80];
	unsigned char uSideIdx, lSideIdx, uSideUsageIdx, lSideUsageIdx;
	unsigned char sanMinutesMax = 0, sanMinutesMin = 0xFF, sanMinutesUpper, sanMinutesLower, uMins, lMins;
	unsigned int uHrs, lHrs;
	
	
	print_ecdbg("\r\n***INSTALLED LED BOARDS***\r\n\r\n");
	
	print_ecdbg(" LED | LED BOARD  |   UPPER SIDE     |   LOWER SIDE    \r\n");
	print_ecdbg("SLOT |    ID      | HRS:MIN    DTE   | HRS:MIN    DTE   \r\n");
	print_ecdbg("--------------------------------------------------------\r\n");
	
	for (int i=0; i<NUM_LED_BOARDS; i++)
	{
		if (ledBrd[i].present)
		{
			uSideIdx = ledBrd[i].uSideIdx;
			lSideIdx = ledBrd[i].lSideIdx;
			
			if (uSideIdx != NO_LED_BOARD_PRESENT)
			{
				uSideUsageIdx = ledBrdSide[uSideIdx].ushdwIdx;	
				ledBrdSide[uSideIdx].maxUsageReached = !check_led_brd_side_lifetime(uSideIdx);
				sanMinutesUpper = ledBrdSide[uSideIdx].sanitizeMinutes;
#ifdef SERIAL_ID_AND_ALL_USAGE_COMBINED
				uHrs = usageShdw[0].u[uSideUsageIdx].minutes/60;
				uMins = usageShdw[0].u[uSideUsageIdx].minutes%60;
#else
				uHrs = ush.u[uSideUsageIdx].hours;
				uMins = um.mins[uSideUsageIdx];
#endif				
			}
			else
			{
				uHrs = 0;
				uMins = 0;
				sanMinutesUpper = 0;
			}
			
			if (lSideIdx != NO_LED_BOARD_PRESENT)
			{
				lSideUsageIdx = ledBrdSide[lSideIdx].ushdwIdx;	
				ledBrdSide[uSideIdx].maxUsageReached = !check_led_brd_side_lifetime(lSideIdx);
				sanMinutesLower = ledBrdSide[lSideIdx].sanitizeMinutes;
#ifdef SERIAL_ID_AND_ALL_USAGE_COMBINED
				lHrs = usageShdw[0].u[lSideUsageIdx].minutes/60;
				lMins = usageShdw[0].u[lSideUsageIdx].minutes%60;
#else
				lHrs = ush.u[lSideUsageIdx].hours;
				lMins = um.mins[lSideUsageIdx];
#endif
			}
			else
			{
				lHrs = 0;
				lMins = 0;
				sanMinutesLower = 0;
			} 
			
			
			sprintf(pStr, "%2d     %X%X%X%X%X%X  %04d:%02d     %02d     %04d:%02d     %02d\r\n", 
				i, 
				ledBrd[i].id[0], ledBrd[i].id[1], ledBrd[i].id[2], ledBrd[i].id[3], ledBrd[i].id[4], ledBrd[i].id[5],
				uHrs, uMins,
				sanMinutesUpper,
				lHrs, lMins,
				sanMinutesLower);
			print_ecdbg(pStr);
			

			/* 
			 * Determine the min and max sanitize times for the LED boards that are currently installed
			 */
			if ((sanMinutesMax < sanMinutesUpper) && (sanMinutesUpper != 0))
			{
				sanMinutesMax = sanMinutesUpper;
			}
			if ((sanMinutesMax < sanMinutesLower) && (sanMinutesLower != 0))
			{
				sanMinutesMax = sanMinutesLower;
			}
			if ((sanMinutesMin > sanMinutesUpper) && (sanMinutesUpper != 0))
			{
				sanMinutesMin = sanMinutesUpper;
			}
			if ((sanMinutesMin > sanMinutesLower) && (sanMinutesLower != 0))
			{
				sanMinutesMin = sanMinutesLower;
			}
		}
	}
	
	print_ecdbg("MAX DTE: ");
	print_ecdbg_num(sanMinutesMax);
	print_ecdbg(" MIN DTE: ");
	print_ecdbg_num(sanMinutesMin);
	print_ecdbg("\r\n");
	
	print_ecdbg("TOTAL SANITIZE HOURS: ");
#ifdef SERIAL_ID_AND_ALL_USAGE_COMBINED	
	if ((usageShdw[0].totalSanitationMinutes%60) > 30)
	{
		print_ecdbg_num((usageShdw[0].totalSanitationMinutes/60)+1);
	}
	else
	{
		print_ecdbg_num(usageShdw[0].totalSanitationMinutes/60);
	}
	print_ecdbg(" TOTAL SANITIZE CYCLES: ");
	print_ecdbg_num(usageShdw[0].totalSanitationCycles);
#else
	if ((sanm.mins%60) > 30)
	{
		print_ecdbg_num((sanm.mins/60)+1);
	}
	else
	{
		print_ecdbg_num(sanm.mins/60);
	}
	print_ecdbg(" TOTAL SANITIZE CYCLES: ");
	print_ecdbg_num(sanc.cycles);

#endif //SERIAL_ID_AND_ALL_USAGE_COMBINED	
	print_ecdbg("\r\n");
	
}


void show_chassis_sysErr(void);
void show_chassis_sysErr(void)
{
	char str[80];
	

	print_ecdbg("\r\n***SYSTEM TESTS***\r\n\r\n");


/*
 *	LED Driver: Top
 */
	sprintf(str, "LED Driver: TOP (7..0)                 ");
	
	for (int i=8; i>0; i--)
	{
		if ((sysErr.topdrive & BIT(i-1)))
		{
			strcat(str,"F ");			
		}
		else
		{
			strcat(str,"P ");
		}
	}
	
	print_ecdbg(str);
	print_ecdbg("\r\n");
	
/*
 *	LED Driver: Bottom
 */
	sprintf(str, "LED Driver: BOTTOM (11..0)             ");
	
	for (int i=12; i>0; i--)
	{
		if ((sysErr.botdrive & BIT(i-1)))
		{
			strcat(str,"F ");			
		}
		else
		{
			strcat(str,"P ");
		}
	}
	
	print_ecdbg(str);
	print_ecdbg("\r\n");
	
/*
 *	Flash
 */

	sprintf(str, "Flash (0..1)                           ");
	
	for (int i=0; i<2; i++)
	{
		if ((sysErr.flashArea & BIT(i)) == SE_FAIL)
		{
			strcat(str, "F ");
		}
		else
		{
			strcat(str, "P ");
		}
	}
	
	print_ecdbg(str);
	print_ecdbg("\r\n");
	
/*
 * LED board serial ID checksums
 */	
	sprintf(str, "LED Board Serial ID Checksums (0..4)   ");
	
	for (int i=0; i<NUM_LED_BOARDS; i++)
	{
		if ((sysErr.ledBrdSerialIdCsum & BIT(i)) == SE_FAIL)
		{
			strcat(str, "F ");
		}
		else
		{
			strcat(str, "P ");
		}
	}

	print_ecdbg(str);
	print_ecdbg("\r\n");
	

/*
 * LED Board Side Max Usage Reached
 */
	sprintf(str, "LED Board Side Max Usage (0..7)        ");
	
	for (int i=0; i<NUM_LED_BOARD_SIDES; i++)
	{
		if (ledBrdSide[i].maxUsageReached)
		{
			strcat(str, "F ");
		}
		else
		{
			strcat(str, "P ");
		}
	}

	print_ecdbg(str);
	print_ecdbg("\r\n");
	

/*
 * Usage Struct Full
 */

	sprintf(str, "Usage Struct Open Slots                ");
	if (sysErr.usageStructsFull == SE_FAIL)
	{
		strcat(str, "F \r\n");
	}
	else
	{
		strcat(str, "P \r\n");
	}
	
	print_ecdbg(str);
	print_ecdbg("\r\n");

}

void show_chassis_all_LED_boards(void);
void show_chassis_all_LED_boards(void)
{
	char str[80];
	int i = 0;

	print_ecdbg("\r\n***LED BOARDS MASTER LIST***\r\n\r\n");
	
	while(1)
	{
#ifdef SERIAL_ID_AND_ALL_USAGE_COMBINED
		if (usageShdw[0].u[i].slotFilled)
		{
			sprintf(str, "%2d) %X%X%X%X%X%X ", i,
				usageShdw[0].u[i].id[0],usageShdw[0].u[i].id[1],usageShdw[0].u[i].id[2],usageShdw[0].u[i].id[3],usageShdw[0].u[i].id[4],usageShdw[0].u[i].id[5]);
			
			if (usageShdw[0].u[i].top_botn)
#else
		if (ush.u[i].slotFilled)
		{
			sprintf(str, "%2d) %X%X%X%X%X%X ", i,
			ush.u[i].id[0],ush.u[i].id[1],ush.u[i].id[2],ush.u[i].id[3],ush.u[i].id[4],ush.u[i].id[5]);
			
			if (ush.u[i].top_botn)

#endif //SERIAL_ID_AND_ALL_USAGE_COMBINED			
			{
				strcat(str, " TOP\r\n");
			}
			else
			{
				strcat(str, " BOT\r\n");
			}
			
			print_ecdbg(str);
		}
		else
		{
			break; //LED boards are stored contiguously, so if we hit a blank spot we are done with the entries in the list
		}
		i++;
		
	}
	
	print_ecdbg("\r\n\r\n");

}




/*! \brief Main File Section:
 *          - Initialization (CPU, TWI, Usart,...)
 */
int main(void)
{
	static unsigned char displayIdx = 0;
	
	// Initialize System Clock
	init_sys_clocks();

	init_io();
	
	init_sysErr();

	init_led_board_info();
	
	//Set clock to 8MHz. We start at 100MHz to get through the DS2411 LED board serial ID detection. But we don't need to run that fast for remaining operations.
	osc_enable(OSC_ID_RC8M);
	osc_wait_ready(OSC_ID_RC8M);
	sysclk_set_source(SYSCLK_SRC_RC8M);
	sysclk_set_prescalers(0,0,0,0);
	pll_disable(0);



	// Initialize USART again after changing the system clock
	init_ecdbg_rs232(FPBA_HZ);
	init_display_rs232(FPBA_HZ);

	// Print Startup Message
	print_ecdbg("\r\nELECTROCLAVE\r\nCopyright (c) 2015 Seal Shield, Inc.\r\n");
	print_ecdbg("Hardware Version: Classic +++ Software Version: 0.005\r\n");

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
	test_led_driver_channels();
	
	
	electroclaveState = STATE_EC_IDLE;
	
	show_chassis_status_info();
	show_chassis_sysErr();
	show_chassis_all_LED_boards();
	
	gpio_set_pin_low(ECLAVE_LED_OEn); //...and we are live!
	gpio_set_pin_low(ECLAVE_PSUPPLY_ONn); //turn the leds on first and then the power supply
	
	cpu_set_timeout(EC_ONE_SECOND/2, &debugTimer);


	// Main loop
	while (true) 
	{

		switch(electroclaveState)
		{
			case STATE_EC_IDLE:
				if (EC_DOOR_LATCHED) {
					gpio_set_pin_low(ECLAVE_DEBUG_LED);
					print_ecdbg("Door latch detected\r\n");
//					display_text(IDX_CLEAR);
					display_text(IDX_READY);
					electroclaveState = STATE_DOOR_LATCHED;
					firstTimeThroughDoorLatch = 1;
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
					cpu_delay_ms(500, EC_CPU_CLOCK_FREQ);
					display_text(IDX_CLEANING);
					cpu_delay_ms(3000, EC_CPU_CLOCK_FREQ); //give display time to update, scroll all the way across
				}
				else {
					electroclaveState = STATE_START_CLEAN;
					print_ecdbg("No shelves, no devices or shelves are past lifetime\r\n");
//					display_text(IDX_CLEAR);
					display_text(IDX_READY);
				}
				break;
				
			case STATE_START_SANITIZE:
				display_text(IDX_CLEAR);
				cpu_delay_ms(500, EC_CPU_CLOCK_FREQ); //half second TODO: figure out why this is here and get rid of it, don't like to just hang for no reason, especially when we need to be monitoring the door latch
				
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
				
				displayTimerSeconds = cpu_ms_2_cy(8000, EC_CPU_CLOCK_FREQ); //8 seconds per "shelf" display is enough time for the text to scroll twice
				cpu_set_timeout(displayTimerSeconds, &displayTimer);
				
#if 0 //DEBUG: set this to seconds not minutes so we can debug this logic faster 11may15				
				cpu_set_timeout((sanitizeMinutes * 60 * cpu_ms_2_cy(1000, EC_CPU_CLOCK_FREQ)), &sanitizeTimer);
#endif
				cpu_set_timeout((sanitizeMinutes * cpu_ms_2_cy(1000, EC_CPU_CLOCK_FREQ)), &sanitizeTimer); //DEBUG take this out when done debugging logic, put it back to minutes 11may15

#ifdef SERIAL_ID_AND_ALL_USAGE_COMBINED
				usageShdw[0].totalSanitationCycles++;
				usageShdw[1].totalSanitationCycles++;
#else

				inc_sanCycles();

#endif //SERIAL_ID_AND_ALL_USAGE_COMBINED
				
//DEBUG 11may15 do this once per second for debug				cpu_set_timeout((60 * cpu_ms_2_cy(1000,EC_CPU_CLOCK_FREQ)), &oneMinuteTimer); //one minute for the usage statistics
				cpu_set_timeout((cpu_ms_2_cy(1000,EC_CPU_CLOCK_FREQ)), &oneMinuteTimer); //one minute for the usage statistics DEBUG 11may15

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
					
//DEBUG 11may15 set to one second for debug					cpu_set_timeout(cpu_ms_2_cy(60000, EC_CPU_CLOCK_FREQ), &oneMinuteTimer); //one minute for the usage statistics
					cpu_set_timeout((cpu_ms_2_cy(1000,EC_CPU_CLOCK_FREQ)), &oneMinuteTimer); //one minute for the usage statistics DEBUG 11may15 one second instead of one minute
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
					electroclaveState = STATE_START_CLEAN;
				}
				break;
				
			case STATE_START_CLEAN:
				display_text(IDX_CLEAN);
				electroclaveState = STATE_CLEAN;
#if 0 //DEBUG do this in seconds to debug logic 11may15				
				cpu_set_timeout((20 * 60 * cpu_ms_2_cy(1000, EC_CPU_CLOCK_FREQ)), &cleanTimer); //TODO: this time period will be parameterized from the technician UART interface
#endif
				cpu_set_timeout((20 * cpu_ms_2_cy(1000, EC_CPU_CLOCK_FREQ)), &cleanTimer); //DEBUG 11may15 

				break;	
				
			case STATE_CLEAN:
				if (cpu_is_timeout(&cleanTimer)) {
					cpu_stop_timeout(&cleanTimer);
					electroclaveState = STATE_ACTION_PB_RELEASED;	
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
		
			if (firstTimeThroughDoorLatch)
			{
				door_latch_open_kill_all_shelves();

				display_text(IDX_CLEAR);
				cpu_delay_ms(500, EC_CPU_CLOCK_FREQ);
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
				firstTimeThroughDoorLatch = 0;
				
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
