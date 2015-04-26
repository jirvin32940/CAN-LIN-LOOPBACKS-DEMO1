/*****************************************************************************
 *
 * \file
 *
 * \brief PCA9952 driver for AVR32 UC3.
 *
 * This file is the PCA9952 driver. 
 *
 * 7apr15 Copied from the touch controller AT42QT1060 for its TWI (I2C) interface and modified to fit the PCA9952 LED Driver.
 *
 * Copyright (c) 2014 Atmel Corporation. All rights reserved.
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

#ifndef _PCA9952_H_
#define _PCA9952_H_

#include <stdint.h>

enum {
	
	LED_OFF,
	LED_ON
};

// List of Control Commands
#define PCA9952_MODE1	0x00
#define PCA9952_MODE2	0x01
#define PCA9952_LEDOUT0	((uint8_t)0x02)
#define PCA9952_LEDOUT1	((uint8_t)0x03)
#define PCA9952_LEDOUT2	((uint8_t)0x04)
#define PCA9952_LEDOUT3	((uint8_t)0x05)

#define PCA9952_GRPPWM	0x08
#define PCA9952_GRPFREQ	0x09
#define PCA9952_PWM00	0x0a
#define PCA9952_PWM01	0x0b
#define PCA9952_PWM02	0x0c
#define PCA9952_PWM03	0x0d
#define PCA9952_PWM04	0x0e
#define PCA9952_PWM05	0x0f
#define PCA9952_PWM06	0x10
#define PCA9952_PWM07	0x11
#define PCA9952_PWM08	0x12
#define PCA9952_PWM09	0x13
#define PCA9952_PWM10	0x14
#define PCA9952_PWM11	0x15
#define PCA9952_PWM12	0x16
#define PCA9952_PWM13	0x17
#define PCA9952_PWM14	0x18
#define PCA9952_PWM15	0x19

#define PCA9952_IREF00	0x22
#define PCA9952_IREF01	0x23
#define PCA9952_IREF02	0x24
#define PCA9952_IREF03	0x25
#define PCA9952_IREF04	0x26
#define PCA9952_IREF05	0x27
#define PCA9952_IREF06	0x28
#define PCA9952_IREF07	0x29
#define PCA9952_IREF08	0x2a
#define PCA9952_IREF09	0x2b
#define PCA9952_IREF10	0x2c
#define PCA9952_IREF11	0x2d
#define PCA9952_IREF12	0x2e
#define PCA9952_IREF13	0x2f
#define PCA9952_IREF14	0x30
#define PCA9952_IREF15	0x31

#define PCA9952_OFFSET		0x3a
#define PCA9952_SUBADR1		0x3b
#define PCA9952_SUBADR2		0x3c
#define PCA9952_SUBADR3		0x3d
#define PCA9952_ALLCALLADR	0x3e
#define PCA9952_RESERVED1	0x3f
#define PCA9952_RESERVED2	0x40
#define PCA9952_RESERVED3	0x41
#define PCA9952_PWMALL		0x42
#define PCA9952_IREFALL		0x43
#define PCA9952_EFLAG0		0x44
#define PCA9952_EFLAG1		0x45


/*! \brief Write data to a register.
 *
 * \param reg_index Register index number
 * \param data Register data
 */
void PCA9952_write_reg(unsigned char topBotn, uint8_t reg_index, uint8_t data);

/*! \brief Read register data.
 *
 * \param reg_index Register index
 * \returns Register value
 */
uint8_t PCA9952_read_reg(unsigned char topBotn, uint8_t reg_index);

/*! \brief Initialize touch sensor with default configuration values.
 *
 */
//7apr15 void PCA9952_init(int32_t fcpu);
void PCA9952_init(void); //7apr15

enum {
	LED_TOP,
	LED_BOTTOM
};

void led_shelf(unsigned char shelf, unsigned char onOffn);


#endif // _PCA9952_H_

