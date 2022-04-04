# SX128X LoRa transceiver driver for [contiki-ng](https://github.com/contiki-ng/contiki-ng) OS

This driver was developped as a means of studying the effectiveness of 
the [TSCH](https://tools.ietf.org/html/rfc7554) protocol in conjunction with LoRa.

## How to use

Clone this repository with the `contiki-ng` submodule.

```
git clone --recursive git@github.com:tperale/sx128x.git
```

There is an exemple of how to use this driver in `/exemple/` folder in this
repository. You can see you need to declare the driver in your `Makefile` by adding
the following line.

```
MODULES += $(CONTIKI_NG_DRIVERS_DIR)/sx128x/src
```

Also be sure to declare the radio driver as your `NETSTACK` radio driver
in the `project-conf.h` file.

```
#define NETSTACK_CONF_RADIO sx128x_radio_driver
```
