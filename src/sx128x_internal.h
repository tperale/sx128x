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
 * @brief       Semtech SX128X internal functions
 *
 * @author      Eugene P. <ep@unwds.com>
 * @author      José Ignacio Alamos <jose.alamos@inria.cl>
 * @author      Alexandre Abadie <alexandre.abadie@inria.fr>
 */

#ifndef SX128X_INTERNAL_H
#define SX128X_INTERNAL_H

#include "sx128x.h"
#include <inttypes.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief   Writes the radio register at specified address.
 *
 * @param[in] dev                      The sx128x device structure pointer
 * @param[in] addr                     Register address
 * @param[in] data                     New register value
 */
void sx128x_reg_write(const sx128x_t *dev, uint16_t addr, uint8_t data);

/**
 * @brief   Reads the radio register at specified address.
 *
 * @param[in] dev                      The sx128x device structure pointer
 * @param[in] addr                     Register address
 *
 * @return	Register value
 */
uint8_t sx128x_reg_read(const sx128x_t *dev, uint16_t addr);

/**
 * @brief
 *
 */
void sx128x_cmd_burst(const sx128x_t *dev, uint8_t cmd, uint8_t *in, 
                      uint8_t in_size, uint8_t *out, uint8_t out_size);

/**
 * @brief   Writes multiple radio registers starting at address (burst-mode).
 *
 * @param[in] dev                      The sx128x device structure pointer
 * @param[in] addr                     First radio register address
 * @param[in] buffer                   Buffer containing the new register's
 * values
 * @param[in] size                     Number of registers to be written
 */
void sx128x_reg_write_burst(const sx128x_t *dev, uint16_t reg, uint8_t *buffer, uint8_t size);

/**
 * @brief   Reads multiple radio registers starting at address.
 *
 * @param[in]  dev                     The sx128x device structure pointer
 * @param[in]  addr                    First radio register address
 * @param[in]  size                    Number of registers to be read
 * @param[out] buffer                  Buffer where to copy registers data
 */
uint8_t sx128x_reg_read_burst(const sx128x_t *dev, uint16_t addr);

/**
 * @brief   Writes the buffer contents to the SX1276 FIFO
 *
 * @param[in] dev                      The sx128x device structure pointer
 * @param[in] buffer                   Buffer Buffer containing data to be put
 * on the FIFO.
 * @param[in] size                     Size Number of bytes to be written to the
 * FIFO
 */
void sx128x_write_fifo(const sx128x_t *dev, uint8_t *buffer, uint8_t size);

/**
 * @brief   Reads the contents of the SX1276 FIFO
 *
 * @param[in] dev                      The sx128x device structure pointer
 * @param[in] size                     Size Number of bytes to be read from the
 * FIFO
 * @param[out] buffer                  Buffer Buffer where to copy the FIFO read
 * data.
 */
void sx128x_read_fifo(const sx128x_t *dev, uint8_t *buffer, uint8_t size);

#ifdef __cplusplus
}
#endif

/* SX128X_INTERNAL_H */
/** @} */
#endif
