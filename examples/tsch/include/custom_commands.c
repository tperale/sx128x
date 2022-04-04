#include "custom_commands.h"

#include "contiki.h"
#include "dev/serial-line.h"
#include "dev/spi.h"
#include "dev/uart.h"
#include "netstack.h"
#include "process.h"
#include "random.h"
#include "rtimer-arch.h"
#include "shell.h"
#include "shell-commands.h"
#include "sx128x.h"
#include "sys/_stdint.h"
#include "sys/log.h"
#include <stdio.h>
#include <stdlib.h>

#define TIMEOUTVALUE (5 * 2000)

#ifndef MAC_CONF_WITH_TSCH
static PT_THREAD(shell_recv(struct pt *pt, shell_output_func output, char *args)) {
  PT_BEGIN(pt);
  char buf[255];
  int timeout = 0;

  NETSTACK_RADIO.on();
  while (!NETSTACK_RADIO.receiving_packet() && !NETSTACK_RADIO.pending_packet() && timeout < TIMEOUTVALUE) {
    watchdog_periodic();
    clock_delay_usec(500);
    timeout++;
  }
  if (!NETSTACK_RADIO.pending_packet()) {
    SHELL_OUTPUT(output, "Waiting for pending\n");
    while (!NETSTACK_RADIO.pending_packet() && timeout < TIMEOUTVALUE) {
      watchdog_periodic();
      clock_delay_usec(500);
      timeout++;
    }
  }

  if (timeout >= TIMEOUTVALUE) {
    NETSTACK_RADIO.off();
    SHELL_OUTPUT(output, "Recv timed out after 10sec\n");
    PT_EXIT(pt);
  }

  int len = NETSTACK_RADIO.read((void *)buf, 255);
  NETSTACK_RADIO.off();
  SHELL_OUTPUT(output, "Received (%d bytes): '%s'\n", len, buf);
  PT_END(pt);
}

static PT_THREAD(shell_send(struct pt *pt, shell_output_func output, char *args)) {
  char *next_args;

  PT_BEGIN(pt);

  SHELL_ARGS_INIT(args, next_args);

  SHELL_ARGS_NEXT(args, next_args);
  if (args == NULL) {
    SHELL_OUTPUT(output, "Sending basic 'helloworld'\n");
    NETSTACK_RADIO.send("helloworld", 10);
    PT_EXIT(pt);
  }

  NETSTACK_RADIO.send(args, strlen(args));

  PT_END(pt);
}

static PT_THREAD(shell_tsym(struct pt *pt, shell_output_func output, char *args)) {
  char *next_args;
  char *sf;
  char *bw;

  PT_BEGIN(pt);

  SHELL_ARGS_INIT(args, next_args);

  SHELL_ARGS_NEXT(sf, next_args);
  if (args == NULL) {
    SHELL_OUTPUT(output, "Should specify <sf> and <bw>.\n");
    PT_EXIT(pt);
  }

  SHELL_ARGS_NEXT(bw, next_args);
  if (args == NULL) {
    SHELL_OUTPUT(output, "Should specify <bw>.\n");
    PT_EXIT(pt);
  }

  SHELL_OUTPUT(output, "%d\n", t_sym(atoi(sf), atoi(bw)));

  PT_END(pt);
}

static PT_THREAD(shell_scan(struct pt *pt, shell_output_func output, char *args)) {
  PT_BEGIN(pt);

  NETSTACK_RADIO.on();
  int i = 0;
  while (i < TIMEOUTVALUE && (NETSTACK_RADIO.receiving_packet() == false)) {
    watchdog_periodic();
    i++;
    int ch = TSCH_DEFAULT_HOPPING_SEQUENCE[(i % TSCH_HOPPING_SEQUENCE_MAX_LEN)];
    NETSTACK_RADIO.set_value(RADIO_PARAM_CHANNEL, ch);
  }

  NETSTACK_RADIO.off();
  if (i >= TIMEOUTVALUE) {
    SHELL_OUTPUT(output, "'scan' command timed out\n");
    PT_EXIT(pt);
  }

  SHELL_OUTPUT(output, "%d\n", i % TSCH_HOPPING_SEQUENCE_MAX_LEN);

  PT_END(pt);
}

static PT_THREAD(shell_ch(struct pt *pt, shell_output_func output, char *args)) {
  PT_BEGIN(pt);
  char *next_args;

  SHELL_ARGS_INIT(args, next_args);

  SHELL_ARGS_NEXT(args, next_args);
  if (args == NULL) {
    SHELL_OUTPUT(output, "Should specify <ch> to switch to.\n");
    PT_EXIT(pt);
  }

  int ch = (atoi(args) % TSCH_HOPPING_SEQUENCE_MAX_LEN);

  NETSTACK_RADIO.set_value(RADIO_PARAM_CHANNEL, TSCH_DEFAULT_HOPPING_SEQUENCE[ch]);

  SHELL_OUTPUT(output, "Changing to channel %d\n", ch);

  PT_END(pt);
}
#endif

static PT_THREAD(shell_toa(struct pt *pt, shell_output_func output, char *args)) {
  char *next_args;

  PT_BEGIN(pt);

  SHELL_ARGS_INIT(args, next_args);

  SHELL_ARGS_NEXT(args, next_args);
  if (args == NULL) {
    SHELL_OUTPUT(output, "Should specify packet length (in bytes)\n");
    PT_EXIT(pt);
  }

  SHELL_OUTPUT(output, "%ld\n", RTIMERTICKS_TO_US_64(TSCH_PACKET_DURATION(atoi(args))));

  PT_END(pt);
}
/* static unsigned long last_tx, last_rx, last_time, last_cpu, last_lpm; */

/* static unsigned long */
/* to_permil(unsigned long delta_metric, unsigned long delta_time) */
/* { */
/*   return (1000ul * (delta_metric)) / delta_time; */
/* } */

/* static unsigned long */
/* to_ms(unsigned long time) */
/* { */
/*   return (unsigned) (time * 1000) / ENERGEST_SECOND; */
/* } */
static PT_THREAD(shell_energest(struct pt *pt, shell_output_func output, char *args)) {
  PT_BEGIN(pt);
  /* static unsigned count = 0; */
  /* energest_flush(); */

  /* unsigned long curr_time = ENERGEST_GET_TOTAL_TIME(); */
  /* unsigned long curr_cpu = energest_type_time(SX127X_ENERGEST_STANDBY); */
  /* unsigned long curr_lpm = energest_type_time(SX127X_ENERGEST_SLEEP); */
  /* unsigned long curr_tx = energest_type_time(SX127X_ENERGEST_TX); */
  /* unsigned long curr_rx = energest_type_time(SX127X_ENERGEST_RX); */

  /* unsigned long delta_time = curr_time - last_time; */
  /* unsigned long delta_cpu = curr_cpu - last_cpu; */
  /* unsigned long delta_lpm = curr_lpm - last_lpm; */
  /* unsigned long delta_tx = curr_tx - last_tx; */
  /* unsigned long delta_rx = curr_rx - last_rx; */

  /* LOG_INFO("--- Period summary #%u (%lu ms)\n", count++, to_ms(delta_time)); */
  /* LOG_INFO("Total time  : %10lu\n", delta_time); */
  /* LOG_INFO("CPU         : %10lu (%lu permil)\n", to_ms(delta_cpu), to_permil(delta_cpu, delta_time)); */
  /* LOG_INFO("LPM         : %10lu (%lu permil)\n", to_ms(delta_lpm), to_permil(delta_lpm, delta_time)); */
  /* LOG_INFO("Radio Tx    : %10lu (%lu permil)\n", to_ms(delta_tx), to_permil(delta_tx, delta_time)); */
  /* LOG_INFO("Radio Rx    : %10lu (%lu permil)\n", to_ms(delta_rx), to_permil(delta_rx, delta_time)); */
  /* LOG_INFO("Radio total : %10lu (%lu permil)\n", to_ms(delta_tx+delta_rx), to_permil(delta_tx+delta_rx, delta_time)); */
  PT_END(pt);
}

static PT_THREAD(shell_record(struct pt *pt, shell_output_func output, char *args)) {
  PT_BEGIN(pt);
  /* last_time = ENERGEST_GET_TOTAL_TIME(); */
  /* last_cpu = energest_type_time(SX127X_ENERGEST_STANDBY); */
  /* last_lpm = energest_type_time(SX127X_ENERGEST_SLEEP); */
  /* last_tx = energest_type_time(SX127X_ENERGEST_TX); */
  /* last_rx = energest_type_time(SX127X_ENERGEST_RX); */
  PT_END(pt);
}

const struct shell_command_t custom_shell_commands[] = {
#ifndef MAC_CONF_WITH_TSCH
    {"send", shell_send, "'> send': Send a basic 'helloworld' message using LoRa Radio."},
    {"recv", shell_recv, "'> recv': Busywait the next message."},
    {"tsym", shell_tsym, "'> tsym <sf> <bw> : Time in 'us' for a symbol to get transmitted depending on <sf> and <bw>."},
    {"scan", shell_scan, "'> scan : Check the channel currently used for transmission."},
    {"ch", shell_ch, "'> ch <ch> : Change transmitting channel."},
#endif
    {"toa", shell_toa, "'> toa <len> : Calculate the Time On Air in 'us' of a packet of <len> bytes."},
    {"energest", shell_energest, "'> energest': ."},
    {"record", shell_record, "'> record': ."},
    {NULL, NULL, NULL},
};

static struct shell_command_set_t custom_shell_command_set = {
    .next = NULL,
    .commands = custom_shell_commands,
};

void shell_custom_init(void) {
  shell_command_set_register(&custom_shell_command_set);
}
