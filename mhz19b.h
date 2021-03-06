/*
 * Copyright (c) 2015, Zolertia - http://www.zolertia.com
 * All rights reserved.
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
/*---------------------------------------------------------------------------*/
/**
 * \addtogroup zoul-sensors
 * @{
 *
 * \defgroup zoul-mhz19b-sensor MH-Z19B Sensor
 *
 * Driver for the MH-Z19B sensor
 *
 *
 * \file
 * Header file for the MH-Z19B Sensor Driver
 * 
 * Lauri Anton <lauri.ntn@gmail.com>
 */

/* 
 * Connect MH-Z19B to Zolertia Re-Mote UART1
 */

/*---------------------------------------------------------------------------*/
#ifndef MHZ19B_H_
#define MHZ19B_H_
#endif /* ifndef MHZ19B_H_ */
/*---------------------------------------------------------------------------*/

#define MHZ19B_BUFF_SIZE 9   /* MH-Z19B response is 9 bytes long */

#define MHZ19B_SHORT_WAIT 12   /* ~0.1 sec */

/** \brief Send bytes via UART to MH-Z19B sensor */
unsigned int 
uart_send_bytes(const uint8_t *buf, uint8_t len);

/** \brief Callback function assigned to UART receiving line */
int uart_rx_callback(unsigned char c);

/** \brief MH-Z19B checksum calculation */
uint8_t mhz19b_checksum(uint8_t *packet);

/** \brief Start reading, callback will finish */
void mhz19b_start_reading(unsigned int* co2);

void co2_reading_callback (void* co2_reading);

/* Init */
void mhz19b_init(uint8_t uart);

