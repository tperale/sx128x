/*
 * Copyright (C) 2017 Inria
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    net_lora LoRa modulation
 * @ingroup     net
 * @brief       LoRa modulation header definitions
 * @{
 *
 * @file
 * @brief       LoRa modulation header definitions
 *
 * @author      Alexandre Abadie <alexandre.abadie@inria.fr>
 */

#ifndef NET_LORA24_H
#define NET_LORA24_H

#include <stdint.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup net_lora_conf  LoRa 2.4GHz modulation compile configurations
 * @ingroup  config
 * @{
 */
/** @brief Frequency resolution in Hz */
#ifndef LORA24_FREQUENCY_RESOLUTION_DEFAULT
#define LORA24_FREQUENCY_RESOLUTION_DEFAULT      (198.3642578)
#endif

/** @brief Preamble length, same for Tx and Rx
 *
 * Configure preamble used in LoRa frame. Each LoRa frame begins with a
 * preamble. It starts with a series of upchirps to cover the whole frequency
 * band of the particular channel assigned. The last two upchirps encode the
 * sync word. Sync word is used to differentiate between LoRa transmissions that
 * use the same frequency bands. The sync word is followed by two and a quarter
 * downchirps, for a duration of 2.25 symbols. The total duration of this
 * preamble can be configured between 10.25 and 65,539.25 symbol hence the value
1* can range from 8 to 65537.
*/
#ifndef CONFIG_LORA24_PREAMBLE_LENGTH_DEFAULT
#define CONFIG_LORA24_PREAMBLE_LENGTH_DEFAULT         (8U)
#endif

/** @brief Symbol timeout period in symbols
 *
 * Configure symbol time out in terms of number of symbols. One symbol has a
 * length in time of (2^SF)/BW seconds.
*/
#ifndef CONFIG_LORA24_SYMBOL_TIMEOUT_DEFAULT
#define CONFIG_LORA24_SYMBOL_TIMEOUT_DEFAULT          (10U)
#endif

/** @brief Set channel bandwidth
 *
 * Configure the channel bandwidth. Refer to country specific regulation on
 * channel usage to identify the correct bandwidth.
*/
/* #if IS_ACTIVE(CONFIG_LORA24_BW_DEFAULT_200) */
/* #define CONFIG_LORA24_BW_DEFAULT                      (LORA24_BW_200_KHZ) */
/* #elif IS_ACTIVE(CONFIG_LORA24_BW_DEFAULT_400) */
/* #define CONFIG_LORA24_BW_DEFAULT                      (LORA24_BW_400_KHZ) */
/* #elif IS_ACTIVE(CONFIG_LORA24_BW_DEFAULT_800) */
/* #define CONFIG_LORA24_BW_DEFAULT                      (LORA24_BW_800_KHZ) */
/* #elif IS_ACTIVE(CONFIG_LORA24_BW_DEFAULT_1600) */
/* #define CONFIG_LORA24_BW_DEFAULT                      (LORA24_BW_1600_KHZ) */
/* #endif */

#ifndef CONFIG_LORA24_BW_DEFAULT
#define CONFIG_LORA24_BW_DEFAULT                      (LORA24_BW_200_KHZ)
#endif

/** @brief Set Spreading Factor (SF)
 *
 * Configure Spreading Factor (SF). SF denotes the amount of spreading code
 * applied to the original data signal. A larger SF increases the time on air,
 * which increases energy consumption, reduces the data rate, and improves
 * communication range. Each step up in spreading factor effectively doubles the
 * time on air to transmit the same amount of data. Refer to country specific
 * air time usage regulations before varying the SF. To calculate air time refer
 * https://www.loratools.nl/#/airtime .
*/
/* #if IS_ACTIVE(CONFIG_LORA24_SF_DEFAULT_SF5) */
/* #define CONFIG_LORA24_SF_DEFAULT                      (LORA24_SF6) */
/* #elif IS_ACTIVE(CONFIG_LORA24_SF_DEFAULT_SF6) */
/* #define CONFIG_LORA24_SF_DEFAULT                      (LORA24_SF6) */
/* #elif IS_ACTIVE(CONFIG_LORA24_SF_DEFAULT_SF7) */
/* #define CONFIG_LORA24_SF_DEFAULT                      (LORA24_SF7) */
/* #elif IS_ACTIVE(CONFIG_LORA24_SF_DEFAULT_SF8) */
/* #define CONFIG_LORA24_SF_DEFAULT                      (LORA24_SF8) */
/* #elif IS_ACTIVE(CONFIG_LORA24_SF_DEFAULT_SF9) */
/* #define CONFIG_LORA24_SF_DEFAULT                      (LORA24_SF9) */
/* #elif IS_ACTIVE(CONFIG_LORA24_SF_DEFAULT_SF10) */
/* #define CONFIG_LORA24_SF_DEFAULT                      (LORA24_SF10) */
/* #elif IS_ACTIVE(CONFIG_LORA24_SF_DEFAULT_SF11) */
/* #define CONFIG_LORA24_SF_DEFAULT                      (LORA24_SF11) */
/* #elif IS_ACTIVE(CONFIG_LORA24_SF_DEFAULT_SF12) */
/* #define CONFIG_LORA24_SF_DEFAULT                      (LORA24_SF12) */
/* #endif */

#ifndef CONFIG_LORA24_SF_DEFAULT
#define CONFIG_LORA24_SF_DEFAULT                      (LORA24_SF7)
#endif

/** @brief Set Coding Rate (CR)
 *
 * Configure Coding Rate (CR). CR denotes the implementation of forward error
 * correction (FEC). This may be done by encoding 4-bit data with redundancies
 * into 5-bit, 6-bit, 7-bit, or 8-bit. Coding Rate (CR) value need to be
 * adjusted according to conditions of the channel used for data transmission.
 * If there are too many interferences in the channel, then it’s recommended to
 * increase the value of CR. However, the rise in CR value will also increase
 * the duration for the transmission. Refer to country specific air time usage
 * regulations before varying the CR. To calculate air time refer
 * https://www.loratools.nl/#/airtime .
*/
/* #if IS_ACTIVE(CONFIG_LORA24_CR_DEFAULT_CR_4_5) */
/* #define CONFIG_LORA24_CR_DEFAULT                      (LORA24_CR_4_5) */
/* #elif IS_ACTIVE(CONFIG_LORA24_CR_DEFAULT_CR_4_6) */
/* #define CONFIG_LORA24_CR_DEFAULT                      (LORA24_CR_4_6) */
/* #elif IS_ACTIVE(CONFIG_LORA24_CR_DEFAULT_CR_4_7) */
/* #define CONFIG_LORA24_CR_DEFAULT                      (LORA24_CR_4_7) */
/* #elif IS_ACTIVE(CONFIG_LORA24_CR_DEFAULT_CR_4_8) */
/* #define CONFIG_LORA24_CR_DEFAULT                      (LORA24_CR_4_8) */
/* #endif */

#ifndef CONFIG_LORA24_CR_DEFAULT
#define CONFIG_LORA24_CR_DEFAULT                      (LORA24_CR_4_5)
#endif

/** @brief Set this to 1 to enable inverted I/Q mode
 *
 * Enable this to invert the IQ signals used in RF modulation circuit. For more
 * information on I/Q modulation technique visit http://www.ni.com/tutorial/4805/en/
*/
#ifdef DOXYGEN
#define CONFIG_LORA24_IQ_INVERTED_DEFAULT
#endif

/** @brief Set this to 1 to enable frequency hopping
 *
 * If Frequency hopping spread spectrum (FHSS) is enabled a portion of each LoRa
 * packet is transmitted on each hopping channel from a look up table of
 * frequencies managed by the host microcontroller.
*/
#ifdef DOXYGEN
#define CONFIG_LORA24_FREQUENCY_HOPPING_DEFAULT
#endif

/** @brief Frequency hopping period in symbols
 *
 * Configure the hopping period, in symbols, time which each transmission will
 * dwell in any given channel. One symbol has a length in time of (2^SF)/BW
 * seconds.
*/
#ifndef CONFIG_LORA24_FREQUENCY_HOPPING_PERIOD_DEFAULT
#define CONFIG_LORA24_FREQUENCY_HOPPING_PERIOD_DEFAULT (0U)
#endif

/** @brief Set this to 1 to enable fixed header length mode (implicit header)
 *
 * If fixed header length mode ( implicit header mode) is enabled, PHY header
 * (`PHDR`) in LoRa frame is discarded. For more information, refer to the
 * section "LoRa frame structure" in this
 * <a href="https://link.springer.com/article/10.1186/s13638-019-1542-x">publication</a>
*/
#ifdef DOXYGEN
#define CONFIG_LORA24_FIXED_HEADER_LEN_MODE_DEFAULT
#endif

/** @brief Enable/disable payload CRC, optional
 *
 * @deprecated Use inverse `CONFIG_LORA_PAYLOAD_CRC_OFF_DEFAULT` instead.
 * Will be removed after 2021.04 release.
*/
#ifndef LORA24_PAYLOAD_CRC_ON_DEFAULT
/* #if IS_ACTIVE(CONFIG_LORA24_PAYLOAD_CRC_OFF_DEFAULT) */
/* #define LORA24_PAYLOAD_CRC_ON_DEFAULT                 (false) */
/* #else */
#define LORA24_PAYLOAD_CRC_ON_DEFAULT                 (true)
/* #endif */
#endif

/** @brief Configure payload length
 *
 * Configure the length of payload. The configuration is unused when using
 * explicit header mode ( @ref CONFIG_LORA_FIXED_HEADER_LEN_MODE_DEFAULT ) as
 * `PHDR` carries the length information.
*/
#ifndef CONFIG_LORA24_PAYLOAD_LENGTH_DEFAULT
#define CONFIG_LORA24_PAYLOAD_LENGTH_DEFAULT          (0U)
#endif
/** @} */

/**
 * @name    LoRa syncword values for network types
 * @{
 */
#define LORA24_SYNCWORD_PUBLIC           (0x34)  /**< Syncword used for public networks */
#define LORA24_SYNCWORD_PRIVATE          (0x12)  /**< Syncword used for private networks */
/** @} */

/**
 * @name    LoRa modulation available values
 *
 */
/**
 * @brief   LoRa modulation bandwidth.
 */
enum {
    LORA24_BW_200_KHZ = 0,               /**< 200 kHz bandwidth */
    LORA24_BW_400_KHZ,                   /**< 400 kHz bandwidth */
    LORA24_BW_800_KHZ,                   /**< 800 kHz bandwidth */
    LORA24_BW_1600_KHZ                   /**< 1600 kHz bandwidth */
};

/**
 * @brief   LoRa modulation spreading factor rate
 */
enum {
    LORA24_SF5 = 5,                      /**< spreading factor 5 */
    LORA24_SF6,                          /**< spreading factor 6 */
    LORA24_SF7,                          /**< spreading factor 7 */
    LORA24_SF8,                          /**< spreading factor 8 */
    LORA24_SF9,                          /**< spreading factor 9 */
    LORA24_SF10,                         /**< spreading factor 10 */
    LORA24_SF11,                         /**< spreading factor 11 */
    LORA24_SF12                          /**< spreading factor 12 */
};

/**
 * @brief   LoRa modulation error coding rate.
 */
enum {
    LORA24_CR_4_5 = 1,                   /**< coding rate 4/5 */
    LORA24_CR_4_6,                       /**< coding rate 4/6 */
    LORA24_CR_4_7,                       /**< coding rate 4/7 */
    LORA24_CR_4_8                        /**< coding rate 4/8 */
};
/** @} */

#define LORA24_BW_TO_KHZ(x) ( \
    (x == LORA24_BW_200_KHZ) ? 200 : ( \
      x == LORA24_BW_400_KHZ ? 400 : ( \
        x == LORA24_BW_800_KHZ ? 800 : 1600 \
      ) \
    ) \
)
#define CEILING_POS(X) ((X-(int)(X)) > 0 ? (int)(X+1) : (int)(X))
#define LORA_T_SYM_USEC(sf, bw) ((1 << (sf)) * 1000 / (LORA24_BW_TO_KHZ(bw)))
#define LORA_SYM_NB_5(sf, crc, header, cr, len) ( \
  6.25 + 8 + CEILING_POS( \
      MAX((float) ( (8 * (len)) - (4 * (sf)) + (16 * (crc)) + (20 * (header)) ), 0) / (4 * (sf)) \
    ) * ((cr) + 4) \
)
#define LORA_SYM_NB_7(sf, crc, header, cr, len) ( \
  4.25 + 8 + CEILING_POS( \
      MAX((float) ( (8 * (len)) - (4 * (sf)) + 8 + (16 * (crc)) + (20 * (header)) ), 0) / (4 * (sf)) \
    ) * ((cr) + 4) \
)
#define LORA_SYM_NB_11(sf, crc, header, cr, len) ( \
  4.25 + 8 + CEILING_POS( \
      MAX((float) ( (8 * (len)) - (4 * (sf)) + 8 + (16 * (crc)) + (20 * (header)) ), 0) / (4 * (sf - 2)) \
    ) * ((cr) + 4) \
)
#define LORA_T_PACKET_USEC(sf, bw, crc, header, cr, prlen, len) ( \
  prlen + \
  ((sf < 7) ? LORA_SYM_NB_5(sf, crc, header, cr, len) : ((sf < 11) ? LORA_SYM_NB_7(sf, crc, header, cr, len) : LORA_SYM_NB_11(sf, crc, header, cr, len))) \
  * LORA_T_SYM_USEC(sf, bw) \
)

#ifdef __cplusplus
}
#endif

#endif /* NET_LORA24_H */
/** @} */
