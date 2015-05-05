/**
 * \file
 *
 * \brief AVR Demo Configuration File
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
 */
 /**
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
 */

#ifndef _CONF_ECLAVE_H_
#define _CONF_ECLAVE_H_

#include "board.h"
#include "conf_clock.h"

#define BUFFER_LENGTH                256

#define DEFAULT_SCREEN_UPDATE_FS_MS  100

/*
 * Bluesense0
 */
#define NUMBER_OF_INPUTS_ADC_SEQ0    1
#define INPUT1_ADC_CHANNEL           0
#define INPUT1_ADC_INP               AVR32_ADCIFA_INP_ADCIN0
#define INPUT1_ADC_INN               AVR32_ADCIFA_INN_GNDANA
#define INPUT1_ADC_PIN               AVR32_ADCIN0_PIN
#define INPUT1_ADC_FUNCTION          AVR32_ADCIN0_FUNCTION

/*
 * Bluesense1
 */
#define NUMBER_OF_INPUTS_ADC_SEQ1    1
#define INPUT2_ADC_CHANNEL           2
#define INPUT2_ADC_INP               AVR32_ADCIFA_INP_ADCIN2
#define INPUT2_ADC_INN               AVR32_ADCIFA_INN_GNDANA
#define INPUT2_ADC_PIN               AVR32_ADCIN2_PIN
#define INPUT2_ADC_FUNCTION          AVR32_ADCIN2_FUNCTION

/*
 * Bluesense2
 */
#define NUMBER_OF_INPUTS_ADC_SEQ2    1
#define INPUT3_ADC_CHANNEL           4
#define INPUT3_ADC_INP               AVR32_ADCIFA_INP_ADCIN4
#define INPUT3_ADC_INN               AVR32_ADCIFA_INN_GNDANA
#define INPUT3_ADC_PIN               AVR32_ADCIN4_PIN
#define INPUT3_ADC_FUNCTION          AVR32_ADCIN4_FUNCTION

/*
 * Bluesense3
 */
#define NUMBER_OF_INPUTS_ADC_SEQ3    1
#define INPUT4_ADC_CHANNEL           6
#define INPUT4_ADC_INP               AVR32_ADCIFA_INP_ADCIN6
#define INPUT4_ADC_INN               AVR32_ADCIFA_INN_GNDANA
#define INPUT4_ADC_PIN               AVR32_ADCIN6_PIN
#define INPUT4_ADC_FUNCTION          AVR32_ADCIN6_FUNCTION


#define TC_INSTANCE                  &AVR32_TC0
#define TC_INSTANCE_IRQ              AVR32_TC0_IRQ0
#define TC_INSTANCE_IRQ_GROUP        AVR32_TC0_IRQ_GROUP
// Timer Counter Channel Used
#define TC_CHANNEL                   0

#endif //_CONF_ECLAVE_H_
