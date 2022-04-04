/*
 * Copyright (c) 2015, SICS Swedish ICT.
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
 */

/**
 * \author Simon Duquennoy <simonduq@sics.se>
 */
#ifndef PROJECT_CONF_H_
#define PROJECT_CONF_H_

#include <stddef.h>

/* Set to enable TSCH security */
#ifndef WITH_SECURITY
#define WITH_SECURITY 0
#endif /* WITH_SECURITY */

/* USB serial takes space, free more space elsewhere */
#define SICSLOWPAN_CONF_FRAG 0
#define UIP_CONF_BUFFER_SIZE 160

/*******************************************************/
/******************* NETSTACK       ********************/
/*******************************************************/

#define NETSTACK_CONF_RADIO sx128x_radio_driver
#define TSCH_CONF_DEFAULT_HOPPING_SEQUENCE                                     \
  (uint8_t[]) { 0, 1, 2 }
extern int tsch_packet_duration(size_t len);
#define TSCH_PACKET_DURATION(len) tsch_packet_duration(len)

/*******************************************************/
/******************* CSMA           ********************/
/*******************************************************/

#define CSMA_CONF_ACK_WAIT_TIME (RTIMER_SECOND / 2)
#define CSMA_CONF_SEND_SOFT_ACK 1

/*******************************************************/
/******************* TSCH Schedule  ********************/
/*******************************************************/

/* 6TiSCH minimal schedule length.
 * Larger values result in less frequent active slots: reduces capacity and
 * saves energy. Configure this variable according to the number of node
 * participating in your network */
#define TSCH_SCHEDULE_CONF_DEFAULT_LENGTH 7
#define TSCH_SCHEDULE_CONF_WITH_6TISCH_MINIMAL 0

/*******************************************************/
/******************* Configure TSCH ********************/
/*******************************************************/

/* IEEE802.15.4 PANID */
#define IEEE802154_CONF_PANID 0x81a5

/* Do not start TSCH at init, wait for NETSTACK_MAC.on() */
#define TSCH_CONF_AUTOSTART 0

#if WITH_SECURITY
/* Enable security */
#define LLSEC802154_CONF_ENABLED 1
#endif /* WITH_SECURITY */

#define TSCH_WAIT_EB RTIMER_SECOND / 2

#define TSCH_CONF_RADIO_ON_DURING_TIMESLOT 0
#define TSCH_CONF_RESYNC_WITH_SFD_TIMESTAMPS 1

#define TSCH_CONF_ADAPTIVE_TIMESYNC 0

#define TSCH_CONF_CHANNEL_SCAN_DURATION CLOCK_SECOND

/* Max time before sending a unicast keep-alive message to the time source */
#define TSCH_CONF_KEEPALIVE_TIMEOUT (48 * CLOCK_SECOND)

/* With TSCH_ADAPTIVE_TIMESYNC enabled: keep-alive timeout used after reaching
 * accurate drift compensation. */
#define TSCH_CONF_MAX_KEEPALIVE_TIMEOUT (240 * CLOCK_SECOND)

/* Max time without synchronization before leaving the PAN */
#define TSCH_CONF_DESYNC_THRESHOLD (4 * TSCH_CONF_KEEPALIVE_TIMEOUT)

/* Period between two consecutive EBs */
#define TSCH_CONF_EB_PERIOD (200 * CLOCK_SECOND)

/* Max Period between two consecutive EBs */
#define TSCH_CONF_MAX_EB_PERIOD (400 * CLOCK_SECOND)
#define TSCH_CONF_MIN_EB_PERIOD (40 * CLOCK_SECOND)

/*******************************************************/
/************* RPL                        **************/
/*******************************************************/

#define RPL_CONF_DIS_INTERVAL (90 * CLOCK_SECOND)
#define RPL_CONF_DAO_RETRANSMISSION_TIMEOUT (90 * CLOCK_SECOND)
#define RPL_CONF_DELAY_BEFORE_LEAVING (300 * CLOCK_SECOND)
#define RPL_CONF_DAO_MAX_RETRANSMISSIONS 15
#define RPL_CONF_DIO_INTERVAL_MIN 15
#define RPL_CONF_DIO_INTERVAL_DOUBLINGS 6
#define NETSTACK_MAX_ROUTE_ENTRIES 5

/*******************************************************/
/************* Logging                    **************/
/*******************************************************/
#define LOG_CONF_OUTPUT_PREFIX(level, levelstr, module) LOG_OUTPUT("[%-4s: %ld.%ld: %-10s] ", levelstr, RTIMER_NOW() / RTIMER_SECOND, ((RTIMER_NOW() % RTIMER_SECOND) * 100) / RTIMER_SECOND, module) 

#define LOG_CONF_LEVEL_RPL LOG_LEVEL_WARN
#define LOG_CONF_LEVEL_TCPIP LOG_LEVEL_WARN
#define LOG_CONF_LEVEL_IPV6 LOG_LEVEL_WARN
#define LOG_CONF_LEVEL_6LOWPAN LOG_LEVEL_WARN
#define LOG_CONF_LEVEL_MAC LOG_LEVEL_INFO
#define LOG_CONF_LEVEL_FRAMER LOG_LEVEL_WARN
#define LOG_CONF_LEVEL_SX128X LOG_LEVEL_INFO
#define LOG_CONF_LEVEL_SX128X_API LOG_LEVEL_INFO
#define TSCH_LOG_CONF_PER_SLOT 1

#endif /* PROJECT_CONF_H_ */
