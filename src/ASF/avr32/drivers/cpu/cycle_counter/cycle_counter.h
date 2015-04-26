/*****************************************************************************
 *
 * \file
 *
 * \brief Cycle counter driver.
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


#ifndef _CYCLE_COUNTER_H_
#define _CYCLE_COUNTER_H_

/**
 * \defgroup group_avr32_drivers_cpu_cycle_counter CPU - Cycle Counter
 *
 * CPU Cycle Counter provides an interface to the COUNT and COMPARE registers.
 *
 * \{
 */

#include "compiler.h"

#define SS_ONE_SECOND 8000000


//! Structure holding private information, automatically initialized by the
//! cpu_set_timeout() function.
typedef struct
{
  //! The cycle count at the beginning of the timeout.
  unsigned long delay_start_cycle;

  //! The cycle count at the end of the timeout.
  unsigned long delay_end_cycle;

  //! Enable/disable the timeout detection
  unsigned char timer_state;
  #define CPU_TIMER_STATE_STARTED 0
  #define CPU_TIMER_STATE_REACHED 1
  #define CPU_TIMER_STATE_STOPPED 2
} t_cpu_time;


/*!
 * \brief Convert milli-seconds into CPU cycles.
 *
 * \param  ms:      Number of millisecond.
 * \param  fcpu_hz: CPU frequency in Hz.
 *
 * \return the converted number of CPU cycles.
 */
__always_inline static uint32_t cpu_ms_2_cy(unsigned long ms, unsigned long fcpu_hz)
{
  return ((unsigned long long)ms * fcpu_hz + 999) / 1000;
}


/*!
 * \brief Convert micro-seconds into CPU cycles.
 *
 * \param  us:      Number of microsecond.
 * \param  fcpu_hz: CPU frequency in Hz.
 *
 * \return the converted number of CPU cycles.
 */
__always_inline static uint32_t cpu_us_2_cy(unsigned long us, unsigned long fcpu_hz)
{
  return ((unsigned long long)us * fcpu_hz + 999999) / 1000000;
}


/*!
 * \brief Convert CPU cycles into milli-seconds.
 *
 * \param  cy:      Number of CPU cycles.
 * \param  fcpu_hz: CPU frequency in Hz.
 *
 * \return the converted number of milli-second.
 */
__always_inline static uint32_t cpu_cy_2_ms(unsigned long cy, unsigned long fcpu_hz)
{
  return ((unsigned long long)cy * 1000 + fcpu_hz-1) / fcpu_hz;
}


/*!
 * \brief Convert CPU cycles into micro-seconds.
 *
 * \param  cy:      Number of CPU cycles.
 * \param  fcpu_hz: CPU frequency in Hz.
 *
 * \return the converted number of micro-second.
 */
__always_inline static uint32_t cpu_cy_2_us(unsigned long cy, unsigned long fcpu_hz)
{
  return ((unsigned long long)cy * 1000000 + fcpu_hz-1) / fcpu_hz;
}


/*!
 * \brief Set a timer variable.
 *
 * Ex:  t_cpu_time timer;
 *      cpu_set_timeout( cpu_ms_2_cy(10, FOSC0), &timer ); // timeout in 10 ms
 *      if( cpu_is_timeout(&timer) )
 *         cpu_stop_timeout(&timer);
 *         ../..
 *
 * \param  delay:   (input) delay in CPU cycles before timeout.
 * \param  cpu_time: (output) internal information used by the timer API.
 */
__always_inline static void cpu_set_timeout(unsigned long delay, t_cpu_time *cpu_time)
{
  cpu_time->delay_start_cycle = Get_system_register(AVR32_COUNT);
  cpu_time->delay_end_cycle   = cpu_time->delay_start_cycle + delay;
  cpu_time->timer_state       = CPU_TIMER_STATE_STARTED;
}


/*!
 * \brief Test if a timer variable reached its timeout.
 *
 * Once the timeout is reached, the function will always return true,
 * until the cpu_stop_timeout() function is called.
 *
 * Ex:  t_cpu_time timer;
 *      cpu_set_timeout( 10, FOSC0, &timer ); // timeout in 10 ms
 *      if( cpu_is_timeout(&timer) )
 *         cpu_stop_timeout(&timer);
 *         ../..
 *
 * \param  cpu_time:   (input) internal information used by the timer API.
 *
 * \return true if timeout occurred, otherwise false.
 */
__always_inline static unsigned long cpu_is_timeout(t_cpu_time *cpu_time)
{
  unsigned long current_cycle_count = Get_system_register(AVR32_COUNT);

  if( cpu_time->timer_state==CPU_TIMER_STATE_STOPPED )
    return false;

  // Test if the timeout as already occurred.
  else if (cpu_time->timer_state == CPU_TIMER_STATE_REACHED)
    return true;

  // If the ending cycle count of this timeout is wrapped, ...
  else if (cpu_time->delay_start_cycle > cpu_time->delay_end_cycle)
  {
    if (current_cycle_count < cpu_time->delay_start_cycle && current_cycle_count > cpu_time->delay_end_cycle)
    {
      cpu_time->timer_state = CPU_TIMER_STATE_REACHED;
      return true;
    }
    return false;
  }
  else
  {
    if (current_cycle_count < cpu_time->delay_start_cycle || current_cycle_count > cpu_time->delay_end_cycle)
    {
      cpu_time->timer_state = CPU_TIMER_STATE_REACHED;
      return true;
    }
    return false;
  }
}


/*!
 * \brief Stop a timeout detection.
 *
 * Ex:  t_cpu_time timer;
 *      cpu_set_timeout( 10, FOSC0, &timer ); // timeout in 10 ms
 *      if( cpu_is_timeout(&timer) )
 *         cpu_stop_timeout(&timer);
 *         ../..
 *
 * \param  cpu_time:   (input) internal information used by the timer API.
 */
__always_inline static void cpu_stop_timeout(t_cpu_time *cpu_time)
{
  cpu_time->timer_state = CPU_TIMER_STATE_STOPPED;
}


/*!
 * \brief Test if a timer is stopped.
 *
 * \param  cpu_time:   (input) internal information used by the timer API.
 *
 * \return true if timer is stopped, otherwise false.
 */
__always_inline static unsigned long cpu_is_timer_stopped(t_cpu_time *cpu_time)
{

  if( cpu_time->timer_state==CPU_TIMER_STATE_STOPPED )
    return true;
  else
    return false;
}


/*!
 * \brief Waits during at least the specified delay (in millisecond) before returning.
 *
 * \param  delay:   Number of millisecond to wait.
 * \param  fcpu_hz: CPU frequency in Hz.
 */
__always_inline static void cpu_delay_ms(unsigned long delay, unsigned long fcpu_hz)
{
  t_cpu_time timer;
  cpu_set_timeout( cpu_ms_2_cy(delay, fcpu_hz), &timer);
  while( !cpu_is_timeout(&timer) );
}

/*!
 * \brief Waits during at least the specified delay (in microsecond) before returning.
 *
 * \param  delay:   Number of microsecond to wait.
 * \param  fcpu_hz: CPU frequency in Hz.
 */
__always_inline static void cpu_delay_us(unsigned long delay, unsigned long fcpu_hz)
{
  t_cpu_time timer;
  cpu_set_timeout( cpu_us_2_cy(delay, fcpu_hz), &timer);
  while( !cpu_is_timeout(&timer) );
}

/*!
 * \brief Waits during at least the specified delay (in CPU cycles) before returning.
 *
 * \param  delay:   Number of CPU cycles to wait.
 */
__always_inline static void cpu_delay_cy(unsigned long delay)
{
  t_cpu_time timer;
  cpu_set_timeout( delay, &timer);
  while( !cpu_is_timeout(&timer) );
}


#define Get_sys_count()     ( Get_system_register(AVR32_COUNT)        )
#define Set_sys_count(x)    ( Set_system_register(AVR32_COUNT,   (x)) )
#define Get_sys_compare()   ( Get_system_register(AVR32_COMPARE)      )
#define Set_sys_compare(x)  ( Set_system_register(AVR32_COMPARE, (x)) )

/**
 * \}
 */

#endif // _CYCLE_COUNTER_H_
