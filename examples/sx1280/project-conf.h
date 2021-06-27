/*---------------------------------------------------------------------------*/
/**
 * \addtogroup zoul-examples
 * @{
 *
 * \file
 * Project specific configuration defines for the basic RE-Mote examples
 */
/*---------------------------------------------------------------------------*/
#ifndef PROJECT_CONF_H_
#define PROJECT_CONF_H_

#include <stddef.h>

#define NETSTACK_CONF_RADIO                        sx128x_radio_driver

extern int tsch_packet_duration(size_t len); 
#define TSCH_PACKET_DURATION(len) tsch_packet_duration(len) 

/* Logging */
#define LOG_CONF_LEVEL_RPL                         LOG_LEVEL_WARN
#define LOG_CONF_LEVEL_TCPIP                       LOG_LEVEL_WARN
#define LOG_CONF_LEVEL_IPV6                        LOG_LEVEL_WARN
#define LOG_CONF_LEVEL_6LOWPAN                     LOG_LEVEL_WARN
#define LOG_CONF_LEVEL_MAC                         LOG_LEVEL_WARN
#define LOG_CONF_LEVEL_FRAMER                      LOG_LEVEL_WARN
#define LOG_CONF_LEVEL_RN2483                      LOG_LEVEL_WARN
#define LOG_CONF_LEVEL_RN2483_UART                 LOG_LEVEL_WARN
#define LOG_CONF_LEVEL_TSCH                        LOG_LEVEL_WARN
#define LOG_CONF_LEVEL_TSCH_LOG                    LOG_LEVEL_WARN
#define TSCH_LOG_CONF_PER_SLOT                     1

#endif /* PROJECT_CONF_H_ */

/** @} */
