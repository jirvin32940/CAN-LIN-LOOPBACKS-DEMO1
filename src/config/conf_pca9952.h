/**
 * \file
 *
 * \brief AVR PCA9952 Configuration File
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
 /**
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
 */
#ifndef _CONF_PCA9952_H_
#define _CONF_PCA9952_H_


#include "board.h" //TODO: ??

	#define PCA9952_DETECT_PIN				AVR32_PIN_PC07 //TODO: ??
	#define PCA9952_TWI						(&AVR32_TWIM0)
	#define PCA9952_TWI_SCL_PIN				AVR32_TWIMS0_TWCK_0_0_PIN
	#define PCA9952_TWI_SCL_FUNCTION		AVR32_TWIMS0_TWCK_0_0_FUNCTION
	#define PCA9952_TWI_SDA_PIN				AVR32_TWIMS0_TWD_0_0_PIN
	#define PCA9952_TWI_SDA_FUNCTION		AVR32_TWIMS0_TWD_0_0_FUNCTION

/*! The PCA9952 can do max 100kHz on the TWI. */
//7apr15 #define PCA9952_TWI_MASTER_SPEED 100000
#define PCA9952_TWI_MASTER_SPEED 400000

/*! The I2C address is fixed for the PCA9952 device. */
//7apr15 #define PCA9952_TWI_ADDRESS               0x12
#define PCA9952_U7_TOPDRIVE_TWI_ADDRESS		0x60  //8apr15 Note that we put the 7 bit address here, don't include a place for R/W, the atmel chip must take care of that somewhere else
#define PCA9952_U8_BOTDRIVE_TWI_ADDRESS     0x61  //8apr15

#endif /* _CONF_PCA9952_H_ */
