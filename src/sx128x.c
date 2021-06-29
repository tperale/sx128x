#include "contiki.h"
#include "lora24.h"
#include "sx128x_registers.h"
#include "sx128x_internal.h"
#include "radio.h"
#include "rtimer-arch.h"
#include "rtimer.h"
#include "spi.h"
#include "sx128x.h"
#include "sys/_stdint.h"
#include "sys/log.h"
#include "os/net/packetbuf.h"
#include "ioc.h"
#include "dev/leds.h"
#include "dev/gpio-hal.h"
#include "net/mac/tsch/tsch.h"
#include "watchdog.h"
#include <stdint.h>

#define LOG_MODULE "SX128X"
#ifndef LOG_CONF_LEVEL_SX128X
#define LOG_CONF_LEVEL_SX128X LOG_LEVEL_DBG
#endif
#define LOG_LEVEL LOG_CONF_LEVEL_SX128X

#define BUFFER_SIZE 256

sx128x_t __sx128x_dev = {
  .settings = {
    .channel = 2400,
    .lora = {
     },
  },
  .params = {
    .spi = {
      .spi_controller = SX128X_SPI_CONTROLLER,
      .pin_spi_sck = GPIO_PORT_PIN_TO_GPIO_HAL_PIN(SX128X_SPI_SCK_PORT, SX128X_SPI_SCK),
      .pin_spi_miso = GPIO_PORT_PIN_TO_GPIO_HAL_PIN(SX128X_SPI_MISO_PORT, SX128X_SPI_MISO),
      .pin_spi_mosi = GPIO_PORT_PIN_TO_GPIO_HAL_PIN(SX128X_SPI_MOSI_PORT, SX128X_SPI_MOSI),
      .pin_spi_cs = GPIO_PORT_PIN_TO_GPIO_HAL_PIN(SX128X_SPI_CS_PORT, SX128X_SPI_CS),
      .spi_bit_rate = SX128X_SPI_BITRATE,
      .spi_pha = SX128X_SPI_PHASE,
      .spi_pol = SX128X_SPI_POL,
    },
    .reset_pin = GPIO_PORT_PIN_TO_GPIO_HAL_PIN(SX128X_RESET_GPIO_PORT, SX128X_RESET_GPIO),
    .busy_pin = GPIO_PORT_PIN_TO_GPIO_HAL_PIN(SX128X_BUSY_PORT, SX128X_BUSY_PIN),
    .dio1_pin =  GPIO_PORT_PIN_TO_GPIO_HAL_PIN(SX128X_DIO1_PORT, SX128X_DIO1_PIN)
  },
  ._internal = {
  },
  .irq = 0,
};

int tsch_packet_duration(size_t len)
{
  return 0; // US_TO_RTIMERTICKS(t_packet(&(SX127X_DEV.lora), len));
} 

/* TSCH timeslot timing (microseconds) */
tsch_timeslot_timing_usec tsch_timing_sx128x = {
  SX128X_TSCH_DEFAULT_TS_CCA_OFFSET, /* tsch_ts_cca_offset */
  SX128X_TSCH_DEFAULT_TS_CCA, /* tsch_ts_cca */
  SX128X_TSCH_DEFAULT_TS_TX_OFFSET, /* tsch_ts_tx_offset */
  SX128X_TSCH_DEFAULT_TS_RX_OFFSET, /* tsch_ts_rx_offset */
  SX128X_TSCH_DEFAULT_TS_RX_ACK_DELAY, /* tsch_ts_rx_ack_delay */
  SX128X_TSCH_DEFAULT_TS_TX_ACK_DELAY, /* tsch_ts_tx_ack_delay */
  SX128X_TSCH_DEFAULT_TS_RX_WAIT, /* tsch_ts_rx_wait */
  SX128X_TSCH_DEFAULT_TS_ACK_WAIT, /* tsch_ts_ack_wait */
  SX128X_TSCH_DEFAULT_TS_RX_TX, /* tsch_ts_rx_tx */
  SX128X_TSCH_DEFAULT_TS_MAX_ACK, /* tsch_ts_max_ack */
  SX128X_TSCH_DEFAULT_TS_MAX_TX, /* tsch_ts_max_tx */
  SX128X_TSCH_DEFAULT_TS_TIMESLOT_LENGTH, /* tsch_ts_timeslot_length */
};

static int sx128x_receiving_packet(void);
static int sx128x_read_packet(void *buf, unsigned short bufsize);

static void sx128x_rx_internal_set(sx128x_t* dev, sx128x_rx_mode rx) {
  switch (rx) {
    case sx128x_rx_receiving:
      if (dev->settings.rx != sx128x_rx_listening) {
        LOG_ERR("[rx_state] Went from '%d' directly to to 'receiving'\n", dev->settings.rx);
      }
      dev->settings.rx = sx128x_rx_receiving;
      break;
    case sx128x_rx_received:
#if SX128X_BUSY_RX
      if (dev->settings.rx != sx128x_rx_receiving) {
        LOG_WARN("[rx_state] Went to 'received' without 'receiving'\n");
      }
#endif
      dev->_internal.pending = true;
      dev->settings.rx = sx128x_rx_received;
      break;
    case sx128x_rx_read:
      if (!dev->_internal.pending) {
        LOG_WARN("[rx_state] read the content of the module without packet pending\n");
      }
      dev->_internal.pending = false;
      break;
    default:
      dev->settings.rx = rx;
  }
}

#if SX128X_USE_INTERRUPT
#define SX128X_DIO1_PORT_BASE GPIO_PORT_TO_BASE(SX128X_DIO1_PORT)
#define SX128X_DIO1_PIN_MASK GPIO_PIN_MASK(SX128X_DIO1_PIN)

#if SX128X_BUSY_RX
static void sx128x_interrupt_dio1(gpio_hal_pin_mask_t pin_mask) { 
}
#else
static void sx128x_interrupt_dio1(gpio_hal_pin_mask_t pin_mask) { 
  SX128X_DEV.irq = sx128x_cmd_get_irq_status(&SX128X_DEV);
  if (sx128x_get_op_mode(&SX128X_DEV) == SX128X_RF_OPMODE_RECEIVER) {
    switch (SX128X_DEV.irq)  {
      case SX128X_IRQ_REG_RX_DONE:
        SX128X_DEV._internal.rx_timestamp = RTIMER_NOW();
        sx128x_cmd_get_packet_status(&SX128X_DEV);
        sx128x_cmd_get_rx_buffer_status(&SX128X_DEV);
        sx128x_rx_internal_set(&SX128X_DEV, sx128x_rx_received);
        sx128x_set_standby(&SX128X_DEV);
        break;
    }

  } else if (sx128x_get_op_mode(&SX128X_DEV) == SX128X_RF_OPMODE_CAD) {
    switch (SX128X_DEV.irq)  {
      case SX128X_IRQ_REG_CAD_DONE:
        sx128x_set_standby(&SX128X_DEV);
        break;
      case SX128X_IRQ_REG_CAD_DETECTED:
        sx128x_set_rx(&SX128X_DEV);
        sx128x_rx_internal_set(&SX128X_DEV, sx128x_rx_receiving);
        break;
    }
  } else if (sx128x_get_op_mode(&SX128X_DEV) == SX128X_RF_OPMODE_TRANSMITTER) {
    switch (SX128X_DEV.irq)  {
      case SX128X_IRQ_REG_TX_DONE:
        sx128x_set_standby(&SX128X_DEV);
        break;
      case SX128X_IRQ_REG_RX_TX_TIMEOUT:
        sx128x_set_standby(&SX128X_DEV);
        break;
    }
  }
  // TODO Handle continuous reception
  /* if (SX128X_DEV.settings.lora.rx_continuous) { */
  /*   packetbuf_clear(); */
  /*   // TODO read the packet length */

  /*   if(len > 0 && len != 3) { */
  /*     sx128x_read_packet(packetbuf_dataptr(), PACKETBUF_SIZE); */
  /*     packetbuf_set_datalen(len); */
  /*     NETSTACK_MAC.input(); */
  /*   } */
  /* } */
  sx128x_cmd_clear_irq_status(&SX128X_DEV);
}
#endif

gpio_hal_event_handler_t sx128x_event_handler_dio1 = {
  .next = NULL,
  .handler = sx128x_interrupt_dio1,
  .pin_mask  = (gpio_hal_pin_to_mask(SX128X_DIO1_PIN) << (SX128X_DIO1_PORT << 3))
  /* .pin_mask  = (gpio_hal_pin_to_mask(GPIO_PORT_PIN_TO_GPIO_HAL_PIN(SX128X_DIO1_PORT, SX128X_DIO1_PIN))) */
};
#endif

static int
sx128x_prepare(const void *payload, unsigned short payload_len) {
  LOG_DBG("Prepare %d bytes\n", payload_len);

  if (sx128x_get_op_mode(&SX128X_DEV) == SX128X_RF_OPMODE_SLEEP || sx128x_get_op_mode(&SX128X_DEV) == SX128X_RF_OPMODE_RECEIVER) {
    sx128x_set_standby(&SX128X_DEV);
    sx128x_rx_internal_set(&SX128X_DEV, sx128x_rx_off);
  }
 
  sx128x_set_payload_length(&SX128X_DEV, payload_len);;
  sx128x_cmd_set_buffer_base_address(&SX128X_DEV, 0, 0);
 
  sx128x_write_fifo(&SX128X_DEV, (uint8_t*) payload, payload_len);

  sx128x_cmd_clear_irq_status(&SX128X_DEV, SX128X_IRQ_REG_ALL);
  sx128x_cmd_set_dio_irq_params(&SX128X_DEV, SX128X_IRQ_REG_TX_DONE | SX128X_IRQ_REG_RX_TX_TIMEOUT, 0, 0);

  return RADIO_RESULT_OK;
}

static int
sx128x_transmit(unsigned short payload_len) {
  sx128x_set_state(&SX128X_DEV, SX128X_RF_TX_RUNNING);
  sx128x_set_op_mode(&SX128X_DEV, SX128X_RF_OPMODE_TRANSMITTER);
  // TODO Set a timeout
  // TODO wait for interrupt
  // TODO Busy wait or interrupt based
  uint16_t irq_reg = 0;
  while(!(irq_reg = sx128x_cmd_get_irq_status(&SX128X_DEV))) {
    clock_delay_usec(1000);
    watchdog_periodic();
  }
  sx128x_cmd_clear_irq_status(&SX128X_DEV, SX128X_IRQ_REG_ALL);
  sx128x_set_standby(&SX128X_DEV);
  if (irq_reg | SX128X_IRQ_REG_TX_DONE) {
    LOG_DBG("Transmited %d bytes with success\n", payload_len);
    return RADIO_TX_OK;
  } else {
    LOG_DBG("Failed to transmit %d bytes\n", payload_len);
    return RADIO_TX_ERR;
  }
}

static int
sx128x_send(const void *payload, unsigned short payload_len) {
  sx128x_prepare(payload, payload_len);
  sx128x_transmit(payload_len);
  return RADIO_TX_OK;
}

static int
sx128x_pending_packet(void) {
#if SX128X_BUSY_RX 
  if (SX128X_DEV.settings.rx == sx128x_rx_received) {
    return true;
  } else if (SX128X_DEV.settings.rx == sx128x_rx_listening) {
    sx128x_receiving_packet();
    return false;
  }

  uint16_t irq_reg = sx128x_cmd_get_irq_status(&SX128X_DEV);
  if (irq_reg | SX128X_IRQ_REG_RX_DONE) {
    sx128x_cmd_get_packet_status(&SX128X_DEV);
    sx128x_cmd_get_rx_buffer_status(&SX128X_DEV);
    sx128x_set_state(&SX128X_DEV, SX128X_RF_IDLE);
    sx128x_rx_internal_set(&SX128X_DEV, sx128x_rx_received);
    sx128x_cmd_clear_irq_status(&SX128X_DEV, SX128X_IRQ_REG_ALL);
  }

#endif

  return SX128X_DEV._internal.pending;
}

static int
sx128x_receiving_packet(void) {
#if SX128X_BUSY_RX
  if (SX128X_DEV.settings.rx == sx128x_rx_receiving) {
    if (sx128x_pending_packet()) {
      return false;
    }
    return true;
  }

  sx128x_set_cad(&SX128X_DEV, SX128X_LORA_CAD_04_SYMBOL);
  uint16_t irq_reg = 0;
  while(!(irq_reg = sx128x_cmd_get_irq_status(&SX128X_DEV))) {
    clock_delay_usec(1000);
    watchdog_periodic();
  }
  if (irq_reg | SX128X_IRQ_REG_CAD_DETECTED) {
    sx128x_set_rx(&SX128X_DEV);
    sx128x_set_op_mode(&SX128X_DEV, SX128X_RF_OPMODE_RECEIVER);
    sx128x_rx_internal_set(&SX128X_DEV, sx128x_rx_receiving);
  } else {
    sx128x_set_standby(&SX128X_DEV);
  }
  sx128x_cmd_clear_irq_status(&SX128X_DEV, SX128X_IRQ_REG_ALL);
#else
  // TODO interrupt based packet detection ?
#endif

  return SX128X_DEV.settings.rx == sx128x_rx_receiving;
}

static int
sx128x_read_packet(void *buf, unsigned short bufsize) {
  if (!sx128x_pending_packet()) {
    return 0;
  }

  // TODO length and SNR should have already been fetched from the pending fn
  // TODO Make sure everything is OK in anycase
  sx128x_read_fifo(&SX128X_DEV, buf, SX128X_DEV._internal.rx_length < bufsize ? SX128X_DEV._internal.rx_length : bufsize);
  if (SX128X_DEV._internal.rx_length < bufsize) {
    ((uint8_t*) buf)[SX128X_DEV._internal.rx_length] = '\0';
  }
 
  LOG_INFO("Received packet of %d bytes\n", SX128X_DEV._internal.rx_length);

  // TODO continuous rx handling
  /* sx128x_rx_internal_set(&SX128X_DEV, sx128x_rx_read); */
  /* if (SX128X_DEV.lora.rx_continuous) { */
  /*   sx128x_rx_internal_set(&SX128X_DEV, sx128x_rx_listening); */
  /*   sx128x_set_op_mode(&SX128X_DEV, sx128x_mode_receiver); */
  /* } else { */
  /*   sx128x_rx_internal_set(&SX128X_DEV, sx128x_rx_off); */
  /*   sx128x_set_op_mode(&SX128X_DEV, sx128x_mode_standby); */
  /* } */

  return SX128X_DEV._internal.rx_length;
}

static int
sx128x_on(void) {
  sx128x_rx_internal_set(&SX128X_DEV, sx128x_rx_listening);
#if !SX128X_BUSY_RX
  // In busy reception mode the RX is triggered with a CAD 
  // scan that is faster to detect the packet than the DIO3 
  // interrupt for the valid header
  // In async reception mode the reception notification will come
  // from the interrupt.
  sx128x_set_op_mode(&SX128X_DEV, sx128x_mode_receiver);
#endif
  return 1;
}

static int
sx128x_off(void) {
  sx128x_set_op_mode(&SX128X_DEV, sx128x_mode_standby);
  sx128x_rx_internal_set(&SX128X_DEV, sx128x_rx_off);
  sx128x_cmd_clear_irq_status(&SX128X_DEV, SX128X_IRQ_REG_ALL);
  sx128x_cmd_set_dio_irq_params(&SX128X_DEV, 0, 0, 0);
  return 1;
}

radio_result_t sx128x_get_value(radio_param_t param, radio_value_t *value){
  if(!value) {
    return RADIO_RESULT_INVALID_VALUE;
  }

  switch(param) {
  case RADIO_PARAM_POWER_MODE:
    *value = SX128X_DEV.settings.opmode == SX128X_RF_OPMODE_STANDBY ? RADIO_POWER_MODE_OFF : RADIO_POWER_MODE_ON;
    return RADIO_RESULT_OK;
  case RADIO_PARAM_CHANNEL:
    switch (sx128x_cmd_get_frequency(&SX128X_DEV)) {
    case 2400:
      *value = 0;
      break;
    }
    return RADIO_RESULT_OK;
  case RADIO_PARAM_RX_MODE:
    if (!sx128x_get_rx_single(&SX128X_DEV)) {
      *value |= RADIO_RX_MODE_POLL_MODE;
    }
    /* if(SX128X_DEV.settings.lora.rx_continuous) { */
    /* } */
    /* if(SX128X_DEV.settings.lora.rx_auto_ack) { */
    /*   *value |= RADIO_RX_MODE_AUTOACK; */
    /* } */
    /* if(SX128X_DEV.settings.lora.rx_address_filter) { */
    /*   *value |= RADIO_RX_MODE_ADDRESS_FILTER; */
    /* } */
    return RADIO_RESULT_OK;
  case RADIO_PARAM_TX_MODE:
    /* if(SX128X_DEV.settings.lora.tx_cca) { */
    /*   *value |= RADIO_TX_MODE_SEND_ON_CCA; */
    /* } */
    return RADIO_RESULT_OK;
  case RADIO_PARAM_TXPOWER:
    *value = SX128X_DEV.settings.lora.power;
    return RADIO_RESULT_OK;
  case RADIO_PARAM_CCA_THRESHOLD:
    /*
     * Clear channel assessment threshold in dBm. This threshold
     * determines the minimum RSSI level at which the radio will assume
     * that there is a packet in the air.
     */
    return RADIO_RESULT_NOT_SUPPORTED;
  case RADIO_PARAM_RSSI:
    /* Return the RSSI value in dBm */
    return RADIO_RESULT_NOT_SUPPORTED;
  case RADIO_PARAM_LAST_RSSI:
    /* RSSI of the last packet received */
    *value = SX128X_DEV._internal.rx_rssi;
    return RADIO_RESULT_OK;
  case RADIO_PARAM_LAST_LINK_QUALITY:
    /* LQI of the last packet received */
    *value = SX128X_DEV._internal.rx_snr;
    return RADIO_RESULT_OK;
  case RADIO_CONST_CHANNEL_MIN:
    *value = 0;
    return RADIO_RESULT_OK;
  case RADIO_CONST_CHANNEL_MAX:
    *value = 2;
    return RADIO_RESULT_OK;
  case RADIO_CONST_TXPOWER_MIN:
    *value = 0;
    return RADIO_RESULT_OK;
  case RADIO_CONST_TXPOWER_MAX:
    *value = 14;
    return RADIO_RESULT_OK;
  case RADIO_CONST_MAX_PAYLOAD_LEN:
    *value = (radio_value_t) 255;
    return RADIO_RESULT_OK;
  case RADIO_CONST_PHY_OVERHEAD:
    *value = sx128x_get_preamble_length(&SX128X_DEV) + (sx128x_get_crc(&SX128X_DEV) ? 5 : 0);
    return RADIO_RESULT_OK;
  case RADIO_CONST_BYTE_AIR_TIME:
    *value = 0;
    return RADIO_RESULT_OK;
  case RADIO_CONST_DELAY_BEFORE_TX:
    *value = 0;
    switch(SX128X_SPI_BITRATE) {
      case 8000000:
        *value = US_TO_RTIMERTICKS(60 // internal time documented in datasheet
            + 122 // time  to switch from standby mode to transmit mode
            + 80
        );
        break;
      default:
        LOG_ERR("Bitrate not supported\n");
    }
    return RADIO_RESULT_OK;
  case RADIO_CONST_DELAY_BEFORE_RX:
    *value = 0;
    switch(SX128X_SPI_BITRATE) {
      case 8000000:
        *value = US_TO_RTIMERTICKS(
            71
            + 153 // Time to set op_mode CAD
        );
        break;
      default:
        LOG_ERR("Bitrate not supported\n");
    }
    return RADIO_RESULT_OK;
  case RADIO_CONST_DELAY_BEFORE_DETECT:
    *value = 0; // US_TO_RTIMERTICKS(2 * t_sym(SX128X_DEV.settings.lora.sf, SX128X_DEV.settings.lora.bw));
    return RADIO_RESULT_OK;
  default:
    return RADIO_RESULT_NOT_SUPPORTED;
  }

  return RADIO_RESULT_OK;
}

/** Set a radio parameter value. */
radio_result_t sx128x_set_value(radio_param_t param, radio_value_t value){
  switch(param) {
  case RADIO_PARAM_POWER_MODE:
    if(value == RADIO_POWER_MODE_ON) {
      sx128x_on();
      return RADIO_RESULT_OK;
    }
    if(value == RADIO_POWER_MODE_OFF) {
      sx128x_off();
      return RADIO_RESULT_OK;
    }
    if(value == RADIO_POWER_MODE_CARRIER_ON ||
       value == RADIO_POWER_MODE_CARRIER_OFF) {
      return RADIO_RESULT_NOT_SUPPORTED;
    }
    return RADIO_RESULT_INVALID_VALUE;
  case RADIO_PARAM_CHANNEL:
    sx128x_set_op_mode(&SX128X_DEV, sx128x_mode_sleep);
    sx128x_cmd_set_frequency(&SX128X_DEV, 2400);
    sx128x_set_op_mode(&SX128X_DEV, sx128x_mode_standby);
    return RADIO_RESULT_OK;
  case RADIO_PARAM_RX_MODE:
    return RADIO_RESULT_OK;
    if(value & ~(RADIO_RX_MODE_ADDRESS_FILTER | RADIO_RX_MODE_AUTOACK | RADIO_RX_MODE_POLL_MODE)) {
      return RADIO_RESULT_INVALID_VALUE;
    }
    /* RN2483_DEV.radio.rx_continuous = (value & RADIO_RX_MODE_POLL_MODE) != 0; */
    /* RN2483_DEV.radio.rx_auto_ack = (value & RADIO_RX_MODE_AUTOACK) != 0; */
    /* RN2483_DEV.radio.rx_address_filter = (value & RADIO_RX_MODE_ADDRESS_FILTER) != 0; */
    return RADIO_RESULT_OK;
  case RADIO_PARAM_TX_MODE:
    /* RN2483_DEV.radio.tx_cca = (value & RADIO_TX_MODE_SEND_ON_CCA) != 0; */
    return RADIO_RESULT_OK;
  case RADIO_PARAM_TXPOWER:
    // TODO verification
    /* if(value < RADIO_PWR_MINUS_3 || value > RADIO_PWR_15) { */
    /*   return RADIO_RESULT_INVALID_VALUE; */
    /* } */
    /* Find the closest higher PA_LEVEL for the desired output power */
    sx128x_set_tx_power(&SX128X_DEV, value);
    return RADIO_RESULT_OK;
  case RADIO_PARAM_CCA_THRESHOLD:
    /*
     * Clear channel assessment threshold in dBm. This threshold
     * determines the minimum RSSI level at which the radio will assume
     * that there is a packet in the air.
     *
     * The CCA threshold must be set to a level above the noise floor of
     * the deployment. Otherwise mechanisms such as send-on-CCA and
     * low-power-listening duty cycling protocols may not work
     * correctly. Hence, the default value of the system may not be
     * optimal for any given deployment.
     */
    return RADIO_RESULT_NOT_SUPPORTED;
  default:
    return RADIO_RESULT_NOT_SUPPORTED;
  }
  return RADIO_RESULT_OK;
}

radio_result_t sx128x_get_object(radio_param_t param, void *dest, size_t size){
  switch(param) {
  case RADIO_PARAM_LAST_PACKET_TIMESTAMP:
    if(size != sizeof(rtimer_clock_t) || !dest) {
      return RADIO_RESULT_INVALID_VALUE;
    }
    /* LOG_DBG("LORA COMM -> %d us of length %d bytes\n", t_packet(&(SX128X_DEV.settings.lora), SX128X_DEV._internal.rx_length), SX128X_DEV._internal.rx_length); */
    /* *(rtimer_clock_t *)dest = SX128X_DEV._internal.rx_timestamp - US_TO_RTIMERTICKS( */
    /*   t_packet(&(SX128X_DEV.settings.lora), SX128X_DEV.settings.rx_length) */
    /*   + 622 // Delay between TX end of transmission and RX detection of end of transmission */
    /*   + 152 // Delay between interrupt on DIO1 and software detection */
    /* ); */
    return RADIO_RESULT_OK;
  case RADIO_CONST_TSCH_TIMING:
    if(size != sizeof(uint32_t *) || !dest) {
      return RADIO_RESULT_INVALID_VALUE;
    }
    *((uint32_t **)dest) = tsch_timing_sx128x;
    return RADIO_RESULT_OK;
  default:
    return RADIO_RESULT_NOT_SUPPORTED;
  }

  return RADIO_RESULT_OK;
}

/**
 * Set a radio parameter object. The memory area referred to by the
 * argument 'src' will not be accessed after the function returns.
 */
radio_result_t sx128x_set_object(radio_param_t param, const void *src, size_t size) {
  return RADIO_RESULT_OK;
}

#define CCA_CLEAR 1
#define CCA_BUSY 0
int sx128x_clear_channel_assesment(){
  return CCA_CLEAR;
}

int sx128x_reset(const sx128x_t *dev) {
  gpio_hal_arch_pin_set_output(0, dev->params.reset_pin);
  gpio_hal_arch_set_pin(0, dev->params.reset_pin);
  clock_delay_usec(20000);
  gpio_hal_arch_clear_pin(0, dev->params.reset_pin);
  clock_delay_usec(50000);
  gpio_hal_arch_set_pin(0, dev->params.reset_pin);
  gpio_hal_arch_pin_set_input(0, dev->params.reset_pin);
  clock_delay_usec(20000);

  return 0;
}

static void sx128x_gpio_init(sx128x_t* dev) {
  gpio_hal_arch_no_port_pin_cfg_set(dev->params.dio1_pin, GPIO_HAL_PIN_CFG_PULL_DOWN);
  gpio_hal_arch_pin_set_input(0, dev->params.dio1_pin);
  gpio_hal_arch_no_port_pin_cfg_set(dev->params.busy_pin, GPIO_HAL_PIN_CFG_PULL_DOWN);
  gpio_hal_arch_pin_set_input(0, dev->params.busy_pin);

#if !SX128X_BUSY_RX
  GPIO_SOFTWARE_CONTROL(SX128X_DIO1_PORT_BASE, SX128X_DIO1_PIN_MASK);
  GPIO_SET_INPUT(SX128X_DIO1_PORT_BASE, SX128X_DIO1_PIN_MASK);
  GPIO_DETECT_EDGE(SX128X_DIO1_PORT_BASE, SX128X_DIO1_PIN_MASK);
  GPIO_DETECT_RISING(SX128X_DIO1_PORT_BASE, SX128X_DIO1_PIN_MASK);
  GPIO_TRIGGER_SINGLE_EDGE(SX128X_DIO1_PORT_BASE, SX128X_DIO1_PIN_MASK);
  ioc_set_over(SX128X_DIO1_PORT, SX128X_DIO1_PIN, IOC_OVERRIDE_DIS);
  gpio_hal_register_handler(&sx128x_event_handler_dio1);
  GPIO_ENABLE_INTERRUPT(SX128X_DIO1_PORT_BASE, SX128X_DIO1_PIN_MASK);

  NVIC_EnableIRQ(GPIO_B_IRQn);
#endif
}

static void sx128x_init_radio(sx128x_t* dev) {
  sx128x_cmd_get_status(dev);
  sx128x_cmd_set_regulator_mode(dev, SX128X_REGULATOR_MODE_DC_DC);
  sx128x_set_standby(dev);
  sx128x_cmd_set_packet_type(dev, SX128X_PACKET_TYPE_DEFAULT);

  sx128x_set_bandwidth(dev, CONFIG_LORA24_BW_DEFAULT);
  sx128x_set_spreading_factor(dev, CONFIG_LORA24_SF_DEFAULT);
  sx128x_set_coding_rate(dev, CONFIG_LORA24_CR_DEFAULT);

  sx128x_set_crc(dev, LORA24_PAYLOAD_CRC_ON_DEFAULT);
  sx128x_set_fixed_header_len_mode(dev, false);
  sx128x_set_iq_invert(dev, false);
  sx128x_set_preamble_length(dev, CONFIG_LORA24_PREAMBLE_LENGTH_DEFAULT);
  sx128x_set_payload_length(dev, CONFIG_LORA24_PAYLOAD_LENGTH_DEFAULT);

  sx128x_cmd_set_frequency(dev, SX128X_CHANNEL_DEFAULT);
  sx128x_cmd_set_buffer_base_address(dev, 0, 0);
  sx128x_set_tx_power(dev, SX128X_RADIO_TX_POWER);
  sx128x_cmd_set_dio_irq_params(dev, 0, 0, 0);
}

int sx128x_initialization() { 
  LOG_DBG("Init SPI\n");
  if (spi_acquire(&SX128X_DEV.params.spi) != SPI_DEV_STATUS_OK) {
    LOG_ERR("Error init SPI\n");
    return RADIO_RESULT_ERROR;
  }

  sx128x_gpio_init(&SX128X_DEV);

  /* LOG_DBG("Reset Module\n"); */
  /* sx128x_reset(&SX128X_DEV); */

  /* sx128x_set_op_mode(&SX128X_DEV, sx128x_mode_sleep); */

  sx128x_init_radio(&SX128X_DEV);

  LOG_INFO("Initialized LoRa module with SF: %d, CR: %d, BW: %d, CRC: %d, PRLEN: %d, HEADER: %d\n", 
      sx128x_get_spreading_factor(&SX128X_DEV), 
      sx128x_get_coding_rate(&SX128X_DEV), 
      sx128x_get_bandwidth(&SX128X_DEV), 
      sx128x_get_crc(&SX128X_DEV), 
      sx128x_get_preamble_length(&SX128X_DEV), 
      sx128x_get_fixed_header_len_mode(&SX128X_DEV)
  );
  LOG_INFO("LoRa driver working with busy RX: %d and interrupt: %d\n", SX128X_BUSY_RX, SX128X_USE_INTERRUPT);
#ifdef MAC_CONF_WITH_TSCH
  LOG_INFO("TX_OFFSET: %d\n", SX128X_TSCH_DEFAULT_TS_TX_OFFSET);
  LOG_INFO("RX_OFFSET: %d\n", SX128X_TSCH_DEFAULT_TS_RX_OFFSET);
  LOG_INFO("TX_ACK_DELAY: %d\n", SX128X_TSCH_DEFAULT_TS_TX_ACK_DELAY);
  LOG_INFO("RX_ACK_DELAY: %d\n", SX128X_TSCH_DEFAULT_TS_RX_ACK_DELAY);
  LOG_INFO("MAX_TX: %d\n", SX128X_TSCH_DEFAULT_TS_MAX_TX);
  LOG_INFO("MAX_ACK: %d\n", SX128X_TSCH_DEFAULT_TS_MAX_ACK);
#endif

  return RADIO_RESULT_OK;
}

const struct radio_driver sx128x_radio_driver = {
  sx128x_initialization,
  sx128x_prepare,
  sx128x_transmit,
  sx128x_send,
  sx128x_read_packet,
  sx128x_clear_channel_assesment,
  sx128x_receiving_packet,
  sx128x_pending_packet,
  sx128x_on,
  sx128x_off,
  sx128x_get_value,
  sx128x_set_value,
  sx128x_get_object,
  sx128x_set_object,
};
