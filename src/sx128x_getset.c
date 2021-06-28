/*
 * Copyright (C) 2016 Unwired Devices <info@unwds.com>
 *               2017 Inria Chile
 *               2017 Inria
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     drivers_sx128x
 * @{
 * @file
 * @brief       Implementation of get and set functions for sx128x
 *
 * @author      Eugene P. <ep@unwds.com>
 * @author      José Ignacio Alamos <jose.alamos@inria.cl>
 * @author      Alexandre Abadie <alexandre.abadie@inria.fr>
 * @}
 */

#include <math.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <inttypes.h>

#include "sys/_stdint.h"
#include "sys/log.h"
#include "lora24.h"

#include "sx128x.h"
#include "sx128x_registers.h"
#include "sx128x_internal.h"

#define LOG_MODULE "SX128X-GETSET"
#ifndef LOG_CONF_LEVEL_SX128X_GETSET
#define LOG_CONF_LEVEL_SX128X_GETSET LOG_LEVEL_INFO
#endif
#define LOG_LEVEL LOG_CONF_LEVEL_SX128X_GETSET


static inline void _set_flag(sx128x_t *dev, uint8_t flag, bool value)
{
    (void)(dev);
    if (value) {
        dev->settings.lora.flags |= flag;
    }
    else {
        dev->settings.lora.flags &= ~flag;
    }
}

uint8_t sx128x_get_state(const sx128x_t *dev)
{
    (void)(dev);
    return dev->settings.state;
}

uint16_t sx128x_get_firmware_version(const sx128x_t *dev)
{
    return (sx128x_reg_read(dev, REG_LR_FIRMWARE_VERSION_MSB) << 8) | sx128x_reg_read(dev, REG_LR_FIRMWARE_VERSION_MSB + 1);
}



uint8_t sx128x_cmd_get_status(const sx128x_t *dev)
{
    uint8_t ret;
    sx128x_cmd_burst(dev, SX128X_CMD_GET_STATUS, NULL, 0, &ret, 1);
    return ret;
}

void sx128x_cmd_write_buffer(const sx128x_t *dev, uint8_t *buf, size_t len)
{
    sx128x_cmd_burst(dev, SX128X_CMD_WRITE_BUF, buf, len, NULL, 0);
}

size_t sx128x_cmd_read_buffer(const sx128x_t *dev, uint8_t *buf, size_t len)
{
    uint8_t offset = 0;
    sx128x_cmd_burst(dev, SX128X_CMD_READ_BUF, &offset, 1, buf, len);
    return 0;
}

void sx128x_cmd_set_sleep(const sx128x_t *dev, uint8_t config) {
    // TODO Check we are on STDBY mode
    LOG_DBG("[sx128x] Cmd set sleep with config %#02x\n", config);
    (void)(dev);
    /* sx128x_cmd_burst(dev, SX128X_CMD_SET_SLEEP, &config, 1, NULL, 0); */
}

void sx128x_cmd_set_standby(const sx128x_t *dev, uint8_t config) {
    LOG_DBG("[sx128x] Cmd set standby with config %#02x\n", config);
    sx128x_cmd_burst(dev, SX128X_CMD_SET_STANDBY, &config, 1, NULL, 0);
}

void sx128x_cmd_set_tx(const sx128x_t *dev, uint8_t period_base, uint16_t period_base_count) {
    LOG_DBG("[sx128x] Cmd set tx with period base %#02x and count %#02x\n", period_base, period_base_count);
    uint8_t params[3] = { period_base, (period_base_count >> 8) & 0xFF, (period_base_count & 0xFF) };
    sx128x_cmd_burst(dev, SX128X_CMD_SET_TX, params, 3, NULL, 0);
}

void sx128x_cmd_set_rx(const sx128x_t *dev, uint8_t period_base, uint16_t period_base_count) {
    LOG_DBG("[sx128x] Set rx\n");
    uint8_t params[3] = { period_base, (period_base_count >> 8) & 0xFF, (period_base_count & 0xFF) };
    sx128x_cmd_burst(dev, SX128X_CMD_SET_RX, params, 3, NULL, 0);
}

void sx128x_cmd_set_cad_params(const sx128x_t *dev, uint8_t symbol_num)
{
    sx128x_cmd_burst(dev, SX128X_CMD_SET_CAD_PARAMS, &symbol_num, 1, NULL, 0);
}

void sx128x_cmd_set_cad(const sx128x_t *dev) {
    sx128x_cmd_burst(dev, SX128X_CMD_SET_CAD, NULL, 0, NULL, 0);
}

void sx128x_cmd_set_packet_type(sx128x_t *dev, uint8_t packet_type) {
    LOG_DBG("[sx128x] Set packet type: %d\n", packet_type);
    if (packet_type != SX128X_PACKET_TYPE_LORA) {
        LOG_DBG("[sx128x] LoRa packet type supported only");
    }
    sx128x_cmd_burst(dev, SX128X_CMD_SET_PACKET_TYPE, &packet_type, 1, NULL, 0);
    dev->settings.modem = packet_type;
}

uint8_t sx128x_cmd_get_packet_type(const sx128x_t *dev) {
    uint8_t ret;
    uint8_t nop = 0;
    sx128x_cmd_burst(dev, SX128X_CMD_GET_PACKET_TYPE, &nop, 1, &ret, 1);
    return ret;
}

void sx128x_cmd_set_regulator_mode(sx128x_t *dev, uint8_t mode) {
    sx128x_cmd_burst(dev, SX128X_CMD_SET_REGULATOR_MODE, &mode, 1, NULL, 0);
}

uint32_t sx128x_cmd_get_frequency(const sx128x_t *dev) {
    return dev->settings.lora.frequency;
}

void sx128x_cmd_set_frequency(sx128x_t *dev, uint32_t freq) {
    // Freq in range (2400..2500)
    dev->settings.lora.frequency = freq;
    uint32_t step = (freq * (1 << SX128X_DIV_EXPONANT)) / SX128X_CRYSTAL_FREQ;
    uint8_t data[3] = {(uint8_t)((step >> 16) & 0xFF), (uint8_t)((step >> 8) & 0xFF), (uint8_t)(step & 0xFF) };
    sx128x_cmd_burst(dev, SX128X_CMD_SET_RF_FREQUENCY, data, 3, NULL, 0);
}

void sx128x_cmd_set_tx_params(const sx128x_t *dev, int8_t power, uint8_t ramp_time) {
    if (power < -18 || power > 13) {
        LOG_DBG("[sx128x] Out of range power range");
        return;
    }
    uint8_t params[2] = {(uint8_t) (power + 18), ramp_time};
    sx128x_cmd_burst(dev, SX128X_CMD_SET_TX_PARAMS, params, 2, NULL, 0);
}

void sx128x_cmd_set_buffer_base_address(const sx128x_t *dev, uint8_t tx_address, uint8_t rx_address)
{
    uint8_t args[2] = {tx_address, rx_address};
    
    sx128x_cmd_burst(dev, SX128X_CMD_SET_BUFFER_BASE_ADDRESS, args, 2, NULL, 0);
}

void sx128x_cmd_set_modulation_params(sx128x_t *dev, uint8_t param1, uint8_t param2, uint8_t param3) {
    uint8_t params[3];
    uint8_t sf_reg = 0;
    
    switch (dev->settings.modem) {
        case SX128X_PACKET_TYPE_LORA:
            dev->settings.lora.datarate = param1;
            switch (param1) {
                case LORA24_SF5:
                    params[0] = SX128X_LORA_SF_5;
                    sf_reg = 0x1E;
                    break;
                case LORA24_SF6:
                    params[0] = SX128X_LORA_SF_6;
                    sf_reg = 0x1E;
                    break;
                case LORA24_SF7:
                    params[0] = SX128X_LORA_SF_7;
                    sf_reg = 0x37;
                    break;
                case LORA24_SF8:
                    params[0] = SX128X_LORA_SF_8;
                    sf_reg = 0x37;
                    break;
                case LORA24_SF9:
                    params[0] = SX128X_LORA_SF_9;
                    sf_reg = 0x32;
                    break;
                case LORA24_SF10:
                    params[0] = SX128X_LORA_SF_10;
                    sf_reg = 0x32;
                    break;
                case LORA24_SF11:
                    params[0] = SX128X_LORA_SF_11;
                    sf_reg = 0x32;
                    break;
                case LORA24_SF12:
                    params[0] = SX128X_LORA_SF_12;
                    sf_reg = 0x32;
                    break;
                default:
                    params[0] = SX128X_LORA_SF_7;
                    sf_reg = 0x37;
                    break;
            }
            dev->settings.lora.bandwidth = param2;
            switch (param2) {
                case LORA24_BW_200_KHZ:
                    params[1] = SX128X_LORA_BW_200;
                    break;
                case LORA24_BW_400_KHZ:
                    params[1] = SX128X_LORA_BW_400;
                    break;
                case LORA24_BW_800_KHZ:
                    params[1] = SX128X_LORA_BW_800;
                    break;
                case LORA24_BW_1600_KHZ:
                    params[1] = SX128X_LORA_BW_1600;
                    break;
                default:
                    params[1] = SX128X_LORA_BW_200;
                    break;
            }
            dev->settings.lora.coderate = param3;
            switch (param3) {
                case LORA24_CR_4_5:
                    params[2] = SX128X_LORA_CR_4_5;
                    break;
                case LORA24_CR_4_6:
                    params[2] = SX128X_LORA_CR_4_6;
                    break;
                case LORA24_CR_4_7:
                    params[2] = SX128X_LORA_CR_4_7;
                    break;
                case LORA24_CR_4_8:
                    params[2] = SX128X_LORA_CR_4_8;
                    break;
                default:
                    params[2] = SX128X_LORA_CR_4_5;
                    break;
            }
            sx128x_cmd_burst(dev, SX128X_CMD_SET_MODULATION_PARAMS, params, 3, NULL, 0);
            if (sf_reg) {
                sx128x_reg_write_burst(dev, 0x925, &sf_reg, 1);
            }
            return;
        default:
            LOG_DBG("[sx128x] Unsupported packet type\n");
            return;
    }
}

void sx128x_cmd_set_packet_params(sx128x_t *dev, uint8_t param1, uint8_t param2, uint8_t param3, uint8_t param4, uint8_t param5, uint8_t param6, uint8_t param7) {
    (void) param6;
    (void) param7;
    uint8_t params[7];
    
    switch (dev->settings.modem) {
        case SX128X_PACKET_TYPE_LORA:
            dev->settings.lora.preamble_len = param1;
            // TODO bit 7:4 represent the exponant in [0:3] * 2^[7:4]
            params[0] = param1;
            _set_flag(dev, SX128X_ENABLE_FIXED_HEADER_LENGTH_FLAG, (bool) param2);
            if (param2) {
                params[1] = SX128X_LORA_IMPLICIT_HEADER;
            } else {
                params[1] = SX128X_LORA_EXPLICIT_HEADER;
            }
            params[2] = param3; // payload length
            _set_flag(dev, SX128X_ENABLE_CRC_FLAG, (bool) param4);
            if (param4) {
                params[3] = SX128X_LORA_CRC_ENABLE;
            } else {
                params[3] = SX128X_LORA_CRC_DISABLE;
            }
            _set_flag(dev, SX128X_IQ_INVERTED_FLAG, (bool) param5);
            if (param5) {
                params[4] = SX128X_LORA_IQ_INVERTED;
            } else {
                params[4] = SX128X_LORA_IQ_STD;
            }
            params[5] = 0;
            params[6] = 0;
            return sx128x_cmd_burst(dev, SX128X_CMD_SET_PACKET_PARAMS, params, 7, NULL, 0);
        default:
            LOG_DBG("[sx128x] Not supported packet type\n");
            return;
    }
}

uint8_t sx128x_cmd_get_rx_buffer_status(const sx128x_t *dev)
{
    (void) dev;
    // TODO
    return 0;
}

uint8_t sx128x_cmd_get_packet_status(const sx128x_t *dev)
{
    (void) dev;
    // TODO
    return 0;
}

void sx128x_cmd_set_dio_irq_params(const sx128x_t *dev, uint16_t dio1_mask, uint16_t dio2_mask, uint16_t dio3_mask) {
    switch (dev->settings.modem) {
        case SX128X_PACKET_TYPE_LORA:
            LOG_DBG("[sx128x] Set irq params\n");
            uint16_t mask = (dio1_mask | dio2_mask | dio3_mask) & SX128X_IRQ_REG_LORA_MASK;
            uint8_t params[8] = {
                (mask & 0xFF00) >> 8, (mask & 0xFF),
                (dio1_mask & 0xFF00) >> 8, (dio1_mask & 0xFF),
                (dio2_mask & 0xFF00) >> 8, (dio2_mask & 0xFF),
                (dio3_mask & 0xFF00) >> 8, (dio3_mask & 0xFF),
            };
            sx128x_cmd_burst(dev, SX128X_CMD_SET_DIO_IRQ_PARAMS, (uint8_t*) params, 8, NULL, 0);
            break;
        default:
            LOG_DBG("[sx128x] Not supported packet type\n");
            break;
    }
}

uint16_t sx128x_cmd_get_irq_status(const sx128x_t *dev)
{
    uint8_t nop = 0;
    uint8_t ret[2];

    sx128x_cmd_burst(dev, SX128X_CMD_GET_IRQ_STATUS, &nop, 1, ret, 2);

    return (ret[0] << 8) | ret[1];
}

void sx128x_cmd_clear_irq_status(const sx128x_t *dev, uint16_t irq_mask)
{
    uint8_t out[2] = {(irq_mask & 0xFF00) >> 8, irq_mask & 0xFF};
    sx128x_cmd_burst(dev, SX128X_CMD_CLR_IRQ_STATUS, out, 2, NULL, 0);
}

void sx128x_set_state(sx128x_t *dev, uint8_t state)
{
    (void)(dev);
    switch (state) {
    case SX128X_RF_IDLE:
        LOG_DBG("[sx128x] Change state: IDLE\n");
        break;
    case SX128X_RF_RX_RUNNING:
        LOG_DBG("[sx128x] Change state: RX\n");
        break;
    case SX128X_RF_TX_RUNNING:
        LOG_DBG("[sx128x] Change state: TX\n");
        break;
    case SX128X_RF_CAD:
        LOG_DBG("[sx128x] Change state: CAD\n");
        break;
    default:
        LOG_DBG("[sx128x] Change state: UNKNOWN\n");
        break;
    }

    dev->settings.state = state;
}

void sx128x_set_modem(sx128x_t *dev, uint8_t modem)
{
    (void)(dev);
    LOG_DBG("[sx128x] set modem: %d\n", modem);
}

uint8_t sx128x_get_syncword(const sx128x_t *dev)
{
    (void)(dev);
    return 0;
}

void sx128x_set_syncword(sx128x_t *dev, uint8_t syncword)
{
    (void)(dev);
    LOG_DBG("[sx128x] Set syncword: %02x\n", syncword);
}

uint32_t sx128x_get_channel(const sx128x_t *dev)
{
    (void)(dev);
    return 0;
}

void sx128x_set_channel(sx128x_t *dev, uint32_t channel)
{
    (void)(dev);
    LOG_DBG("[sx128x] Set channel: %" PRIu32 "\n", channel);

}

uint32_t sx128x_get_time_on_air(const sx128x_t *dev, uint8_t pkt_len)
{
    (void)(dev);
    (void)(pkt_len);
    uint32_t air_time = 0;

    return air_time;
    return LORA_T_PACKET_USEC(
            sx128x_get_spreading_factor(dev), 
            sx128x_get_bandwidth(dev),
            sx128x_get_crc(dev),
            !sx128x_get_fixed_header_len_mode(dev),
            sx128x_get_coding_rate(dev), 
            sx128x_get_preamble_length(dev), 
            pkt_len);
}

void sx128x_set_sleep(sx128x_t *dev)
{
    (void)(dev);
    LOG_DBG("[sx128x] Set sleep\n");

    /* Disable running timers */
    /* ztimer_remove(ZTIMER_MSEC, &dev->_internal.tx_timeout_timer); */
    /* ztimer_remove(ZTIMER_MSEC, &dev->_internal.rx_timeout_timer); */

    /* Put chip into sleep */
    sx128x_set_op_mode(dev, SX128X_RF_OPMODE_SLEEP);
    sx128x_set_state(dev,  SX128X_RF_IDLE);
}

void sx128x_set_standby(sx128x_t *dev)
{
    (void)(dev);
    LOG_DBG("[sx128x] Set standby\n");

    /* Disable running timers */
    /* ztimer_remove(ZTIMER_MSEC, &dev->_internal.tx_timeout_timer); */
    /* ztimer_remove(ZTIMER_MSEC, &dev->_internal.rx_timeout_timer); */

    sx128x_set_op_mode(dev, SX128X_RF_OPMODE_STANDBY);
    sx128x_set_state(dev,  SX128X_RF_IDLE);
}

void sx128x_set_tx(sx128x_t *dev)
{
#ifdef SX128X_USE_RX_SWITCH
    gpio_clear(dev->params.rx_switch_pin);
#endif
#ifdef SX128X_USE_TX_SWITCH
    gpio_set(dev->params.tx_switch_pin);
#endif

    sx128x_cmd_clear_irq_status(dev, SX128X_IRQ_REG_ALL);
    switch (dev->settings.modem) {
        case SX128X_PACKET_TYPE_LORA:
        {
            sx128x_cmd_set_dio_irq_params(dev, SX128X_IRQ_REG_TX_DONE, SX128X_IRQ_REG_RX_TX_TIMEOUT, 0);
            break;
        }
        default:
            LOG_DBG("[sx128x] Unsupported packet type\n");
            break;
    }

    sx128x_set_state(dev, SX128X_RF_TX_RUNNING);

    /* Start TX timeout timer */
    if (dev->settings.lora.tx_timeout != 0) {
        /* ztimer_set(ZTIMER_MSEC, &(dev->_internal.tx_timeout_timer), dev->settings.lora.tx_timeout); */
    }

    /* Put chip into transfer mode */
    sx128x_set_op_mode(dev, SX128X_RF_OPMODE_TRANSMITTER);
}

void sx128x_set_rx(sx128x_t *dev)
{
#ifdef SX128X_USE_TX_SWITCH
    gpio_clear(dev->params.tx_switch_pin);
#endif
#ifdef SX128X_USE_RX_SWITCH
    gpio_clear(dev->params.rx_switch_pin);
#endif

    sx128x_cmd_set_buffer_base_address(dev, 0, 0);
    sx128x_cmd_clear_irq_status(dev, SX128X_IRQ_REG_ALL);
    switch (dev->settings.modem) {
        case SX128X_PACKET_TYPE_LORA:
        {
            sx128x_cmd_set_dio_irq_params(dev, SX128X_IRQ_REG_RX_DONE, SX128X_IRQ_REG_SYNC_WORD_ERROR, SX128X_IRQ_REG_CRC_ERROR);
            break;
        }
        default:
            LOG_DBG("[sx128x] Unsupported packet type\n");
            break;
    }

    sx128x_set_state(dev, SX128X_RF_RX_RUNNING);

    /* Start RX timeout timer */
    if (dev->settings.lora.rx_timeout != 0) {
        /* ztimer_set(ZTIMER_MSEC, &(dev->_internal.rx_timeout_timer), dev->settings.lora.rx_timeout); */
    }

    if (dev->settings.lora.flags & SX128X_RX_CONTINUOUS_FLAG) {
        sx128x_set_op_mode(dev, SX128X_RF_LORA_OPMODE_RECEIVER);
    }
    else {
        sx128x_set_op_mode(dev, SX128X_RF_LORA_OPMODE_RECEIVER_SINGLE);
    }
}

void sx128x_set_cad(sx128x_t *dev, uint8_t cad_symbols)
{
    sx128x_cmd_clear_irq_status(dev, SX128X_IRQ_REG_ALL);
    switch (dev->settings.modem) {
        case SX128X_PACKET_TYPE_LORA:
        {
            sx128x_cmd_set_dio_irq_params(dev, SX128X_IRQ_REG_CAD_DONE | SX128X_IRQ_REG_CAD_DETECTED, 0, 0);
            break;
        }
        default:
            LOG_DBG("[sx128x] Unsupported packet type\n");
            return;
    }

    sx128x_set_state(dev, SX128X_RF_CAD);

    /* Start RX timeout timer */
    if (dev->settings.lora.rx_timeout != 0) {
        /* ztimer_set(ZTIMER_MSEC, &(dev->_internal.rx_timeout_timer), dev->settings.lora.rx_timeout); */
    }

    sx128x_cmd_set_cad_params(dev, cad_symbols);
    sx128x_set_op_mode(dev, SX128X_RF_LORA_OPMODE_CAD);
}



uint8_t sx128x_get_max_payload_len(const sx128x_t *dev)
{
    (void)(dev);
    return 0;
}

void sx128x_set_max_payload_len(const sx128x_t *dev, uint8_t maxlen)
{
    (void)(dev);
    LOG_DBG("[sx128x] Set max payload len: %d\n", maxlen);

}

uint8_t sx128x_get_op_mode(const sx128x_t *dev)
{
    return dev->settings.opmode;
}

void sx128x_set_op_mode(sx128x_t *dev, uint8_t op_mode)
{
    dev->settings.opmode = op_mode;
    switch(op_mode) {
    case SX128X_RF_OPMODE_SLEEP:
        LOG_DBG("[sx128x] Set op mode: SLEEP\n");
        sx128x_cmd_set_sleep(dev, 0);
        break;
    case SX128X_RF_OPMODE_STANDBY:
        LOG_DBG("[sx128x] Set op mode: STANDBY\n");
        sx128x_cmd_set_standby(dev, 0);
        break;
    case SX128X_RF_OPMODE_RECEIVER_SINGLE:
        LOG_DBG("[sx128x] Set op mode: RECEIVER SINGLE\n");
        sx128x_cmd_set_rx(dev, 0, 0);
        break;
    case SX128X_RF_OPMODE_RECEIVER:
        LOG_DBG("[sx128x] Set op mode: RECEIVER\n");
        sx128x_cmd_set_rx(dev, 0, 0);
        break;
    case SX128X_RF_OPMODE_TRANSMITTER:
        LOG_DBG("[sx128x] Set op mode: TRANSMITTER\n");
        sx128x_cmd_set_tx(dev, 0, 0);
        break;
    case SX128X_RF_LORA_OPMODE_CAD:
        LOG_DBG("[sx128x] Set op mode: CAD\n");
        sx128x_cmd_set_cad(dev);
        break;
    default:
        LOG_DBG("[sx128x] Set op mode: UNKNOWN (%d)\n", op_mode);
        break;
    }
}

uint8_t sx128x_get_bandwidth(const sx128x_t *dev)
{
    (void)(dev);
    return dev->settings.lora.bandwidth;
}

void sx128x_set_bandwidth(sx128x_t *dev, uint8_t bandwidth)
{
    (void)(dev);
    LOG_DBG("[sx128x] Set bandwidth: %d\n", bandwidth);
    sx128x_cmd_set_modulation_params(dev, sx128x_get_spreading_factor(dev), bandwidth, sx128x_get_coding_rate(dev));
}

uint8_t sx128x_get_spreading_factor(const sx128x_t *dev)
{
    return dev->settings.lora.datarate;
}

void sx128x_set_spreading_factor(sx128x_t *dev, uint8_t datarate)
{
    (void)(dev);
    LOG_DBG("[sx128x] Set spreading factor: %d\n", datarate);
    sx128x_cmd_set_modulation_params(dev, datarate, sx128x_get_bandwidth(dev), sx128x_get_coding_rate(dev));
}

uint8_t sx128x_get_coding_rate(const sx128x_t *dev)
{
    (void)(dev);
    return dev->settings.lora.coderate;
}

void sx128x_set_coding_rate(sx128x_t *dev, uint8_t coderate)
{
    (void)(dev);
    LOG_DBG("[sx128x] Set coding rate: %d\n", coderate);
    sx128x_cmd_set_modulation_params(dev, sx128x_get_spreading_factor(dev), sx128x_get_bandwidth(dev), coderate);
}

bool sx128x_get_rx_single(const sx128x_t *dev)
{
    (void)(dev);
    return !(dev->settings.lora.flags & SX128X_RX_CONTINUOUS_FLAG);
}

void sx128x_set_rx_single(sx128x_t *dev, bool single)
{
    (void)(dev);
    LOG_DBG("[sx128x] Set RX single: %d\n", single);
    _set_flag(dev, SX128X_RX_CONTINUOUS_FLAG, !single);
}

bool sx128x_get_crc(const sx128x_t *dev)
{
    (void)(dev);
    return true;
    return (dev->settings.lora.flags & SX128X_ENABLE_CRC_FLAG);
}

void sx128x_set_crc(sx128x_t *dev, bool crc)
{
    (void)(dev);
    LOG_DBG("[sx128x] Set CRC: %d\n", crc);
    _set_flag(dev, SX128X_ENABLE_CRC_FLAG, crc);
    sx128x_cmd_set_packet_params(dev, sx128x_get_preamble_length(dev),
            sx128x_get_fixed_header_len_mode(dev), 0, 
            crc, sx128x_get_iq_invert(dev), 0, 0);
}

uint8_t sx128x_get_hop_period(const sx128x_t *dev)
{
    (void)(dev);
    return 0;
}

void sx128x_set_hop_period(sx128x_t *dev, uint8_t hop_period)
{
    (void)(dev);
    LOG_DBG("[sx128x] Set Hop period: %d\n", hop_period);

    dev->settings.lora.freq_hop_period = hop_period;
    // TODO
}

bool sx128x_get_fixed_header_len_mode(const sx128x_t *dev)
{
    (void)(dev);
    return dev->settings.lora.flags & SX128X_ENABLE_FIXED_HEADER_LENGTH_FLAG;
}

void sx128x_set_fixed_header_len_mode(sx128x_t *dev, bool fixed_len)
{
    (void)(dev);
    LOG_DBG("[sx128x] Set fixed header length: %d\n", fixed_len);

    _set_flag(dev, SX128X_ENABLE_FIXED_HEADER_LENGTH_FLAG, fixed_len);
    sx128x_cmd_set_packet_params(dev, sx128x_get_preamble_length(dev), fixed_len,
            0, sx128x_get_crc(dev), sx128x_get_iq_invert(dev), 0, 0);
}

uint8_t sx128x_get_payload_length(const sx128x_t *dev)
{
    (void)(dev);
    return 0;
}

void sx128x_set_payload_length(sx128x_t *dev, uint8_t len)
{
    (void)(dev);
    LOG_DBG("[sx128x] Set payload len: %d\n", len);

    sx128x_cmd_set_packet_params(dev, sx128x_get_preamble_length(dev),
            sx128x_get_fixed_header_len_mode(dev), len, 
            sx128x_get_crc(dev), sx128x_get_iq_invert(dev), 0, 0);
}

static inline uint8_t sx128x_get_pa_select(const sx128x_t *dev)
{
    (void)(dev);
    return 0;
}

uint8_t sx128x_get_tx_power(const sx128x_t *dev)
{
    (void)(dev);
    return dev->settings.lora.power;
}

void sx128x_set_tx_power(sx128x_t *dev, int8_t power)
{
    (void)(dev);
    LOG_DBG("[sx128x] Set power: %d\n", power);

    dev->settings.lora.power = power;
    sx128x_cmd_set_tx_params(dev, power, 0xE0); // TODO ramp time
}

uint16_t sx128x_get_preamble_length(const sx128x_t *dev)
{
    (void)(dev);
    return dev->settings.lora.preamble_len;
}

void sx128x_set_preamble_length(sx128x_t *dev, uint16_t preamble)
{
    (void)(dev);
    LOG_DBG("[sx128x] Set preamble length: %d\n", preamble);

    dev->settings.lora.preamble_len = preamble;
    sx128x_cmd_set_packet_params(dev, preamble,
            sx128x_get_fixed_header_len_mode(dev), 0, 
            sx128x_get_crc(dev), sx128x_get_iq_invert(dev), 0, 0);
}

void sx128x_set_rx_timeout(sx128x_t *dev, uint32_t timeout)
{
    (void)(dev);
    LOG_DBG("[sx128x] Set RX timeout: %" PRIu32 "\n", timeout);

    dev->settings.lora.rx_timeout = timeout;
}

void sx128x_set_tx_timeout(sx128x_t *dev, uint32_t timeout)
{
    (void)(dev);
    LOG_DBG("[sx128x] Set TX timeout: %" PRIu32 "\n", timeout);

    dev->settings.lora.tx_timeout = timeout;
}

void sx128x_set_symbol_timeout(sx128x_t *dev, uint16_t timeout)
{
    (void)(dev);
    LOG_DBG("[sx128x] Set symbol timeout: %d\n", timeout);
}

bool sx128x_get_iq_invert(const sx128x_t *dev)
{
    (void)(dev);
    return dev->settings.lora.flags & SX128X_IQ_INVERTED_FLAG;
}

void sx128x_set_iq_invert(sx128x_t *dev, bool iq_invert)
{
    (void)(dev);
    LOG_DBG("[sx128x] Set IQ invert: %d\n", iq_invert);

    _set_flag(dev, SX128X_IQ_INVERTED_FLAG, iq_invert);
    sx128x_cmd_set_packet_params(dev, sx128x_get_preamble_length(dev),
            sx128x_get_fixed_header_len_mode(dev), 0, 
            sx128x_get_crc(dev), iq_invert, 0, 0);
}

void sx128x_set_freq_hop(sx128x_t *dev, bool freq_hop_on)
{
    (void)(dev);
    LOG_DBG("[sx128x] Set freq hop: %d\n", freq_hop_on);

    _set_flag(dev, SX128X_CHANNEL_HOPPING_FLAG, freq_hop_on);
}
