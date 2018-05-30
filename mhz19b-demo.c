/*
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * This file is part of the Contiki operating system.
 *
 */
/**
 * Proof-of-concept for using CO2 sensor MH-Z19B with Zolertia RE-Mote over UART
 * based on cc2538 demo
 *
 * \author
 *         Lauri Anton <lauri.ntn@gmail.com>
 */
/*---------------------------------------------------------------------------*/
#include "contiki.h"
#include "sys/etimer.h"
#include "dev/leds.h"
#include "dev/uart.h"
#include "dev/mhz19b.h"
#include <stdio.h>
#include <stdint.h>
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
PROCESS(mhz19b_uart_demo_process, "CO2 sensor MH-Z19B via UART demo");
AUTOSTART_PROCESSES(&mhz19b_uart_demo_process);
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
PROCESS_THREAD(mhz19b_uart_demo_process, ev, data)
{
  static struct etimer et, et_short;
  static unsigned int my_co2_reading = 0;

  PROCESS_BEGIN();

  printf("UART1_CONF_BAUD_RATE %d\n", UART1_CONF_BAUD_RATE);

  mhz19b_init(SERIAL_LINE_CONF_UART);

  printf("etimer_set\n");
  etimer_set(&et, CLOCK_SECOND*10);
  etimer_set(&et_short, MHZ19B_SHORT_WAIT*2);  /* ~0.2 sec */

  while(1) {

    mhz19b_start_reading(&my_co2_reading);
    etimer_restart(&et_short);
    PROCESS_WAIT_UNTIL(etimer_expired(&et_short));

    printf("CO2 reading %d\n", my_co2_reading);

    PROCESS_WAIT_UNTIL(etimer_expired(&et));
    etimer_reset(&et);
     
  }

  PROCESS_END();
}
