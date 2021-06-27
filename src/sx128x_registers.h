/*
 * Copyright (C) 2016 Unwired Devices <info@unwds.com>
 *               2016 Inria Chile
 *               2017 Inria
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     drivers_sx127x
 * @{
 *
 * @file
 * @brief       SX128X registers
 *
 * @author      Eugene P. <ep@unwds.com>
 * @author      José Ignacio Alamos <jose.alamos@inria.cl>
 * @author      Alexandre Abadie <alexandre.abadie@inria.fr>
 *
 */

#ifndef SX128X_REGISTERS_H
#define SX128X_REGISTERS_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @name   SX128X transceiver available commands
 * @{
 */

#define SX128X_CMD_GET_STATUS (0xC0)
#define SX128X_CMD_WRITE_REG (0x18)
#define SX128X_CMD_READ_REG (0x19)
#define SX128X_CMD_WRITE_BUF (0x1A)
#define SX128X_CMD_READ_BUF  (0x1B)
#define SX128X_CMD_SET_SLEEP (0x84)
#define SX128X_CMD_SET_STANDBY (0x80)
#define SX128X_CMD_SET_FS (0xC1)
#define SX128X_CMD_SET_TX (0x83)
#define SX128X_CMD_SET_RX (0x82)
#define SX128X_CMD_SET_RX_DUTY_CYCLE (0x94)
#define SX128X_CMD_SET_CAD (0xC5)
#define SX128X_CMD_SET_TX_CONTINUOUS_WAVE (0xD1)
#define SX128X_CMD_SET_TX_CONTINUOUS_PREAMBLE (0xD2)
#define SX128X_CMD_SET_PACKET_TYPE (0x8A)
#define SX128X_CMD_GET_PACKET_TYPE (0x03)
#define SX128X_CMD_SET_RF_FREQUENCY (0x86)
#define SX128X_CMD_SET_TX_PARAMS (0x8E)
#define SX128X_CMD_SET_CAD_PARAMS (0x88)
#define SX128X_CMD_SET_BUFFER_BASE_ADDRESS (0x8F)
#define SX128X_CMD_SET_MODULATION_PARAMS (0x8B)
#define SX128X_CMD_SET_PACKET_PARAMS (0x8C)
#define SX128X_CMD_GET_RX_BUFFER_STATUS (0x17)
#define SX128X_CMD_GET_PACKET_STATUS (0x1D)
#define SX128X_CMD_GET_RSSILNST (0x1F)
#define SX128X_CMD_SET_DIO_IRQ_PARAMS (0x8D)
#define SX128X_CMD_GET_IRQ_STATUS (0x15)
#define SX128X_CMD_CLR_IRQ_STATUS (0x97)
#define SX128X_CMD_SET_REGULATOR_MODE (0x96)
#define SX128X_CMD_SET_SAVE_CONTEXT (0xD5)
#define SX128X_CMD_SET_AUTO_FS (0x9E)
#define SX128X_CMD_SET_AUTO_TX (0x98)
#define SX128X_CMD_SET_PERF_COUNTER_MODE (0x9C)
#define SX128X_CMD_SET_LONG_PREAMBLE (0x9B)
#define SX128X_CMD_SET_UART_SPEED (0x9D)
#define SX128X_CMD_SET_RANGING_ROLE (0xA3)
/** @} */

/**
 * @name   SX128X commands bit definition
 * @{
 */

/* Status settings */
#define SX128X_STATUS_CIRCUIT_MODE_MASK_OFFSET (5)
#define SX128X_STATUS_CIRCUIT_MODE_MASK (0b111 << SX128X_STATUS_CIRCUIT_MODE_MASK_OFFSET)
#define SX128X_STATUS_CIRCUIT_MODE_STDBY_RC (0x2 << SX128X_STATUS_CIRCUIT_MODE_MASK_OFFSET)
#define SX128X_STATUS_CIRCUIT_MODE_STDBY_XOSC (0x3 << SX128X_STATUS_CIRCUIT_MODE_MASK_OFFSET)
#define SX128X_STATUS_CIRCUIT_MODE_FS (0x4 << SX128X_STATUS_CIRCUIT_MODE_MASK_OFFSET)
#define SX128X_STATUS_CIRCUIT_MODE_RX (0x5 << SX128X_STATUS_CIRCUIT_MODE_MASK_OFFSET)
#define SX128X_STATUS_CIRCUIT_MODE_TX (0x6 << SX128X_STATUS_CIRCUIT_MODE_MASK_OFFSET)

#define SX128X_STATUS_COMMAND_STATUS_MASK_OFFSET (2)
#define SX128X_STATUS_COMMAND_STATUS_MASK (0b111 << SX128X_STATUS_COMMAND_STATUS_MASK_OFFSET)
#define SX128X_STATUS_COMMAND_STATUS_SUCCESS (0x1 << SX128X_STATUS_COMMAND_STATUS_MASK_OFFSET)
#define SX128X_STATUS_COMMAND_STATUS_DATA_AVAILABLE (0x2 << SX128X_STATUS_COMMAND_STATUS_MASK_OFFSET)
#define SX128X_STATUS_COMMAND_STATUS_TIMEOUT (0x3 << SX128X_STATUS_COMMAND_STATUS_MASK_OFFSET)
#define SX128X_STATUS_COMMAND_STATUS_PROCESSING_ERROR (0x4 << SX128X_STATUS_COMMAND_STATUS_MASK_OFFSET)
#define SX128X_STATUS_COMMAND_STATUS_FAILURE (0x5 << SX128X_STATUS_COMMAND_STATUS_MASK_OFFSET)
#define SX128X_STATUS_COMMAND_STATUS_TX_DONE (0x6 << SX128X_STATUS_COMMAND_STATUS_MASK_OFFSET)

/* Sleep settings */
#define SX128X_SLEEP_DATA_RAM_RETENTION (0x01)
#define SX128X_SLEEP_DATA_BUFFER_RETENTION (0x02)
#define SX128X_SLEEP_INSTRUCTION_RAM_RETENTION (0x04)

/* Standby settings */
#define SX128X_STANDBY_RC (0x00) /* Device running on RC 13MHz */
#define SX128X_STANDBY_XOSC (0x01) /* Device running on XTAL 52MHz */

/* TX params settings */
#define SX128X_TX_PERIOD_BASE_15_US  (0x00)
#define SX128X_TX_PERIOD_BASE_62_US  (0x01)
#define SX128X_TX_PERIOD_BASE_01_MS  (0x02)
#define SX128X_TX_PERIOD_BASE_04_MS  (0x03)

/* TX params settings */
#define SX128X_TX_RADIO_RAMP_02_US  (0x00)
#define SX128X_TX_RADIO_RAMP_04_US  (0x20)
#define SX128X_TX_RADIO_RAMP_06_US  (0x40)
#define SX128X_TX_RADIO_RAMP_08_US  (0x60)
#define SX128X_TX_RADIO_RAMP_10_US  (0x80)
#define SX128X_TX_RADIO_RAMP_12_US  (0xA0)
#define SX128X_TX_RADIO_RAMP_16_US  (0xC0)
#define SX128X_TX_RADIO_RAMP_20_US  (0xE0)

/* CAD symbol number */
#define SX128X_LORA_CAD_01_SYMBOL  (0x00)
#define SX128X_LORA_CAD_02_SYMBOLS (0x20)
#define SX128X_LORA_CAD_04_SYMBOL (0x40)
#define SX128X_LORA_CAD_08_SYMBOL (0x60)
#define SX128X_LORA_CAD_16_SYMBOL (0x80)

/* Modulation Parameters Spreading Factor */
#define SX128X_LORA_SF_5 (0x50)
#define SX128X_LORA_SF_6 (0x60)
#define SX128X_LORA_SF_7 (0x70)
#define SX128X_LORA_SF_8 (0x80)
#define SX128X_LORA_SF_9 (0x90)
#define SX128X_LORA_SF_10 (0xA0)
#define SX128X_LORA_SF_11 (0xB0)
#define SX128X_LORA_SF_12 (0xC0)

/* Modulation Parameters Bandwidth */
#define SX128X_LORA_BW_1600 (0x0A)
#define SX128X_LORA_BW_800 (0x18)
#define SX128X_LORA_BW_400 (0x26)
#define SX128X_LORA_BW_200 (0x34)

/* Modulation Parameters Coding Rate */
#define SX128X_LORA_CR_4_5 (0x01)
#define SX128X_LORA_CR_4_6 (0x02)
#define SX128X_LORA_CR_4_7 (0x03)
#define SX128X_LORA_CR_4_8 (0x04)
#define SX128X_LORA_CR_LI_4_5 (0x05)
#define SX128X_LORA_CR_LI_4_6 (0x06)
#define SX128X_LORA_CR_LI_4_7 (0x07)

/* Packet type header definition */
#define SX128X_LORA_EXPLICIT_HEADER (0x00)
#define SX128X_LORA_IMPLICIT_HEADER (0x80)

/* Packet type CRC Enabling */
#define SX128X_LORA_CRC_ENABLE (0x20)
#define SX128X_LORA_CRC_DISABLE (0x00)

/* Packet type IQ Swapping */
#define SX128X_LORA_IQ_STD (0x40)
#define SX128X_LORA_IQ_INVERTED (0x00)

/* IRQ Register */
#define SX128X_IRQ_REG_TX_DONE_OFFSET (0x00)
#define SX128X_IRQ_REG_TX_DONE (1 << SX128X_IRQ_REG_TX_DONE_OFFSET)
#define SX128X_IRQ_REG_RX_DONE_OFFSET (0x01)
#define SX128X_IRQ_REG_RX_DONE (1 << SX128X_IRQ_REG_RX_DONE_OFFSET)
#define SX128X_IRQ_REG_SYNC_WORD_VALID_OFFSET (0x02)
#define SX128X_IRQ_REG_SYNC_WORD_VALID (1 << SX128X_IRQ_REG_SYNC_WORD_VALID_OFFSET)
#define SX128X_IRQ_REG_SYNC_WORD_ERROR_OFFSET (0x03)
#define SX128X_IRQ_REG_SYNC_WORD_ERROR (1 << SX128X_IRQ_REG_SYNC_WORD_ERROR_OFFSET)
#define SX128X_IRQ_REG_HEADER_VALID_OFFSET (0x04)
#define SX128X_IRQ_REG_HEADER_VALID (1 << SX128X_IRQ_REG_HEADER_VALID_OFFSET)
#define SX128X_IRQ_REG_HEADER_ERROR_OFFSET (0x05)
#define SX128X_IRQ_REG_HEADER_ERROR (1 << SX128X_IRQ_REG_HEADER_ERROR_OFFSET)
#define SX128X_IRQ_REG_CRC_ERROR_OFFSET (0x06)
#define SX128X_IRQ_REG_CRC_ERROR (1 << SX128X_IRQ_REG_CRC_ERROR_OFFSET)
#define SX128X_IRQ_REG_RANGING_SLAVE_RESPONSE_DONE_OFFSET (0x07)
#define SX128X_IRQ_REG_RANGING_SLAVE_RESPONSE_DONE (1 << SX128X_IRQ_REG_RANGING_SLAVE_RESPONSE_DONE_OFFSET)
#define SX128X_IRQ_REG_RANGING_SLAVE_REQUEST_DISCARD_OFFSET (0x08)
#define SX128X_IRQ_REG_RANGING_SLAVE_REQUEST_DISCARD (1 << SX128X_IRQ_REG_RANGING_SLAVE_REQUEST_DISCARD_OFFSET)
#define SX128X_IRQ_REG_RANGING_MASTER_RESULT_VALID_OFFSET (0x09)
#define SX128X_IRQ_REG_RANGING_MASTER_RESULT_VALID (1 << SX128X_IRQ_REG_RANGING_MASTER_RESULT_VALID_OFFSET)
#define SX128X_IRQ_REG_RANGING_MASTER_TIMEOUT_OFFSET (0x0A)
#define SX128X_IRQ_REG_RANGING_MASTER_TIMEOUT (1 << SX128X_IRQ_REG_RANGING_MASTER_TIMEOUT_OFFSET)
#define SX128X_IRQ_REG_RANGING_MASTER_REQUEST_VALID_OFFSET (0x0B)
#define SX128X_IRQ_REG_RANGING_MASTER_REQUEST_VALID (1 << SX128X_IRQ_REG_RANGING_MASTER_REQUEST_VALID_OFFSET)
#define SX128X_IRQ_REG_CAD_DONE_OFFSET (0x0C)
#define SX128X_IRQ_REG_CAD_DONE (1 << SX128X_IRQ_REG_CAD_DONE_OFFSET)
#define SX128X_IRQ_REG_CAD_DETECTED_OFFSET (0x0D)
#define SX128X_IRQ_REG_CAD_DETECTED (1 << SX128X_IRQ_REG_CAD_DETECTED_OFFSET)
#define SX128X_IRQ_REG_RX_TX_TIMEOUT_OFFSET (0x0E)
#define SX128X_IRQ_REG_RX_TX_TIMEOUT (1 << SX128X_IRQ_REG_RX_TX_TIMEOUT_OFFSET)
#define SX128X_IRQ_REG_PREAMBLE_DETECTED_OFFSET (0x0F)
#define SX128X_IRQ_REG_PREAMBLE_DETECTED (1 << SX128X_IRQ_REG_PREAMBLE_DETECTED_OFFSET)
#define SX128X_IRQ_REG_ALL (0xFFFF)

#define SX128X_IRQ_REG_LORA_MASK (SX128X_IRQ_REG_TX_DONE | SX128X_IRQ_REG_RX_DONE | SX128X_IRQ_REG_HEADER_VALID \
            | SX128X_IRQ_REG_HEADER_ERROR | SX128X_IRQ_REG_CRC_ERROR | SX128X_IRQ_REG_RANGING_SLAVE_REQUEST_DISCARD \
            | SX128X_IRQ_REG_CAD_DONE | SX128X_IRQ_REG_CAD_DETECTED | SX128X_IRQ_REG_RX_TX_TIMEOUT)

/* Regulator mode */
#define SX128X_REGULATOR_MODE_LDO (0x00)
#define SX128X_REGULATOR_MODE_DC_DC (0x01)

/*!
 * \brief Compensation delay for SetAutoTx method in microseconds
 */
#define AUTO_TX_OFFSET                              33

/*!
 * \brief The address of the register holding the firmware version MSB
 */
#define REG_LR_FIRMWARE_VERSION_MSB                 0x0153

/*!
 * \brief The address of the register holding the first byte defining the CRC seed
 *
 * \remark Only used for packet types GFSK and Flrc
 */
#define REG_LR_CRCSEEDBASEADDR                      0x09C8
 
/*!
 * \brief The address of the register holding the first byte defining the CRC polynomial
 *
 * \remark Only used for packet types GFSK and Flrc
 */
#define REG_LR_CRCPOLYBASEADDR                      0x09C6
 
/*!
 * \brief The address of the register holding the first byte defining the whitening seed
 *
 * \remark Only used for packet types GFSK, FLRC and BLE
 */
#define REG_LR_WHITSEEDBASEADDR                     0x09C5
 
/*!
 * \brief The address of the register holding the ranging id check length
 *
 * \remark Only used for packet type Ranging
 */
#define REG_LR_RANGINGIDCHECKLENGTH                 0x0931
 
/*!
 * \brief The address of the register holding the device ranging id
 *
 * \remark Only used for packet type Ranging
 */
#define REG_LR_DEVICERANGINGADDR                    0x0916
 
/*!
 * \brief The address of the register holding the device ranging id
 *
 * \remark Only used for packet type Ranging
 */
#define REG_LR_REQUESTRANGINGADDR                   0x0912
 
/*!
 * \brief The address of the register holding ranging results configuration
 * and the corresponding mask
 *
 * \remark Only used for packet type Ranging
 */
#define REG_LR_RANGINGRESULTCONFIG                  0x0924
#define MASK_RANGINGMUXSEL                          0xCF
 
/*!
 * \brief The address of the register holding the first byte of ranging results
 * Only used for packet type Ranging
 */
#define REG_LR_RANGINGRESULTBASEADDR                0x0961
 
/*!
 * \brief The address of the register allowing to read ranging results
 *
 * \remark Only used for packet type Ranging
 */
#define REG_LR_RANGINGRESULTSFREEZE                 0x097F
 
/*!
 * \brief The address of the register holding the first byte of ranging calibration
 *
 * \remark Only used for packet type Ranging
 */
#define REG_LR_RANGINGRERXTXDELAYCAL                0x092C
 
/*!
 *\brief The address of the register holding the ranging filter window size
 *
 * \remark Only used for packet type Ranging
 */
#define REG_LR_RANGINGFILTERWINDOWSIZE              0x091E
 
/*!
 *\brief The address of the register to reset for clearing ranging filter
 *
 * \remark Only used for packet type Ranging
 */
#define REG_LR_RANGINGRESULTCLEARREG                0x0923
 
 
#define REG_RANGING_RSSI                            0x0964
 
/*!
 * \brief The default number of samples considered in built-in ranging filter
 */
#define DEFAULT_RANGING_FILTER_SIZE                 127
 
/*!
 * \brief The address of the register holding LORA packet parameters
 */
#define REG_LR_PACKETPARAMS                         0x903
 
/*!
 * \brief The address of the register holding payload length
 *
 * \remark Do NOT try to read it directly. Use GetRxBuffer( ) instead.
 */
#define REG_LR_PAYLOADLENGTH                        0x901
 
/*!
 * \brief The addresses of the registers holding SyncWords values
 *
 * \remark The addresses depends on the Packet Type in use, and not all
 *         SyncWords are available for every Packet Type
 */
#define REG_LR_SYNCWORDBASEADDRESS1                 0x09CE
#define REG_LR_SYNCWORDBASEADDRESS2                 0x09D3
#define REG_LR_SYNCWORDBASEADDRESS3                 0x09D8
 
/*!
 * \brief The MSB address and mask used to read the estimated frequency
 * error
 */
#define REG_LR_ESTIMATED_FREQUENCY_ERROR_MSB        0x0954
#define REG_LR_ESTIMATED_FREQUENCY_ERROR_MASK       0x0FFFFF
 
/*!
 * \brief Defines how many bit errors are tolerated in sync word detection
 */
#define REG_LR_SYNCWORDTOLERANCE                    0x09CD
 
/*!
 * \brief Register and mask for GFSK and BLE preamble length forcing
 */
#define REG_LR_PREAMBLELENGTH                       0x09C1
#define MASK_FORCE_PREAMBLELENGTH                   0x8F
 
/*!
 * \brief Register for MSB Access Address (BLE)
 */
#define REG_LR_BLE_ACCESS_ADDRESS                   0x09CF
#define BLE_ADVERTIZER_ACCESS_ADDRESS               0x8E89BED6
 
/*!
 * \brief Select high sensitivity versus power consumption
 */
#define REG_LNA_REGIME                              0x0891
#define MASK_LNA_REGIME                             0xC0
 
/*
 * \brief Register and mask controling the enabling of manual gain control
 */
#define REG_ENABLE_MANUAL_GAIN_CONTROL     0x089F
#define MASK_MANUAL_GAIN_CONTROL           0x80
 
/*!
 * \brief Register and mask controling the demodulation detection
 */
#define REG_DEMOD_DETECTION                0x0895
#define MASK_DEMOD_DETECTION               0xFE
 
/*!
 * Register and mask to set the manual gain parameter
 */
#define REG_MANUAL_GAIN_VALUE              0x089E
#define MASK_MANUAL_GAIN_VALUE             0xF0

#define SX128X_RF_LORA_OPMODE_MASK                                   (0xF8)
#define SX128X_RF_LORA_OPMODE_SLEEP                                  (0x00)
#define SX128X_RF_LORA_OPMODE_STANDBY                                (0x01) /* Default */
#define SX128X_RF_LORA_OPMODE_SYNTHESIZER_TX                         (0x02)
#define SX128X_RF_LORA_OPMODE_TRANSMITTER                            (0x03)
#define SX128X_RF_LORA_OPMODE_SYNTHESIZER_RX                         (0x04)
#define SX128X_RF_LORA_OPMODE_RECEIVER                               (0x05)

/* LoRa specific modes */
#define SX128X_RF_LORA_OPMODE_RECEIVER_SINGLE                        (0x06)
#define SX128X_RF_LORA_OPMODE_CAD                                    (0x07)

#define SX128X_RF_OPMODE_MASK                                        (0xF8)
#define SX128X_RF_OPMODE_SLEEP                                       (0x00)
#define SX128X_RF_OPMODE_STANDBY                                     (0x01)  /* Default */
#define SX128X_RF_OPMODE_SYNTHESIZER_TX                              (0x02)
#define SX128X_RF_OPMODE_TRANSMITTER                                 (0x03)
#define SX128X_RF_OPMODE_SYNTHESIZER_RX                              (0x04)
#define SX128X_RF_OPMODE_RECEIVER                                    (0x05)
#define SX128X_RF_OPMODE_RECEIVER_SINGLE                             (0x06)

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* SX128X_REGISTERS_H */
/** @} */
