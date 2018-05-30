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
#include "sys/ctimer.h"
#include "dev/uart.h"
#include "mhz19b.h"
#include <stdio.h>
#include <stdint.h>
/*---------------------------------------------------------------------------*/


static uint8_t rxbuf_data[ MHZ19B_BUFF_SIZE+1 ];
static unsigned int rxbuf_ptr = 0;
static uint8_t new_reading_ready = 0;
static struct ctimer short_timer;

/* CO2 read command */
uint8_t mhz19b_read_command[] = { 0xFF, 1, 0x86, 0, 0, 0, 0, 0, 0x79 };


/*---------------------------------------------------------------------------*/
unsigned int
uart_send_bytes(const uint8_t *buf, uint8_t len)
{
  uint8_t i = 0;

  if (buf != 0) { 
    for (i = 0; i < len; i++) {
      uart_write_byte(SERIAL_LINE_CONF_UART, buf[i]);
    }
  }
  return i;
}

int uart_rx_callback(unsigned char c)
{
  if(rxbuf_ptr < MHZ19B_BUFF_SIZE) 
  {
    rxbuf_data[rxbuf_ptr++] = c;
  } 

  /* If buffer is filled, then lets reset the rxbuf_ptr */
  if (rxbuf_ptr == MHZ19B_BUFF_SIZE) 
  {
    new_reading_ready = 1; /* TRUE */
    rxbuf_ptr = 0;
    return 1;    /* wake up CPU */

  } else if (rxbuf_ptr > MHZ19B_BUFF_SIZE) 
  {
    printf("Error, rxbuf_ptr %d larger than MHZ19B_BUFF_SIZE %d, should never happen!\n", rxbuf_ptr, MHZ19B_BUFF_SIZE);
    return 1;    /* wake up CPU */
  }

  return 0;      /* waking up CPU is not necessary */
}

/* calculate checksum function based on MH-Z19B datasheet */

uint8_t mhz19b_checksum(uint8_t *packet) {
  uint8_t i;
  uint8_t checksum = 0;
  for (i=1; i<8; i++) {
    checksum += packet[i];
  }
  checksum = 0xFF - checksum;
  checksum += 1;
  return checksum;
}

void mhz19b_init(uint8_t uart) {

  uart_init( uart );
  uart_set_input( uart, uart_rx_callback );

}

void mhz19b_start_reading(unsigned int* co2) {

  uart_send_bytes(mhz19b_read_command, MHZ19B_BUFF_SIZE);
  ctimer_set(&short_timer, MHZ19B_SHORT_WAIT, co2_reading_callback, co2); /* ~0.1 sec */

}

void co2_reading_callback (void* ptr) {
  uint8_t i = 0;
  unsigned int *co2_reading = ptr;

  if (new_reading_ready) {
    if (mhz19b_checksum(rxbuf_data) == rxbuf_data[8]) {
      *co2_reading = rxbuf_data[2] * 256 + rxbuf_data[3];   /* real thing */
    } else {
      printf("CRC error!\n");
    }
  } else {
    printf("Error, reading is not ready! -------------\n");
    *co2_reading = 0;
  }

 /* cleanup operations for the next reading */
  for (i = 0; i < MHZ19B_BUFF_SIZE; i++) {
    rxbuf_data[i] = 0;    
  }
  new_reading_ready = 0;
  rxbuf_ptr = 0;

}


