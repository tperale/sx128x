CONTIKI_PROJECT = node coordinator
all: $(CONTIKI_PROJECT)

TARGET = zoul
BOARD = firefly

ROOT = ../..
CONTIKI = $(ROOT)/contiki-ng

include $(CONTIKI)/Makefile.dir-variables
MODULES += ../src/ # SX128X Driver
MODULES += $(CONTIKI_NG_SERVICES_DIR)/shell
MODULES += ../examples/tsch/include
CFLAGS += -I/usr/arm-none-eabi/include

 # force Orchestra from command line
MAKE_WITH_ORCHESTRA ?= 0
# RPL storing mode?
MAKE_WITH_STORING_ROUTING ?= 0
# Orchestra link-based rule? (Works only if Orchestra & storing mode routing is enabled)
MAKE_WITH_LINK_BASED_ORCHESTRA ?= 0

# MAKE_MAC = MAKE_MAC_CSMA
MAKE_MAC = MAKE_MAC_TSCH
# MAKE_MAC = MAKE_MAC_NULLMAC
MAKE_ROUTING = MAKE_ROUTING_RPL_LITE
# MAKE_ROUTING = MAKE_ROUTING_NULLROUTING

ifeq ($(MAKE_WITH_ORCHESTRA),1)
  MODULES += $(CONTIKI_NG_SERVICES_DIR)/orchestra

  ifeq ($(MAKE_WITH_STORING_ROUTING),1)
    ifeq ($(MAKE_WITH_LINK_BASED_ORCHESTRA),1)
      # enable the `link_based` rule
      CFLAGS += -DORCHESTRA_CONF_RULES="{&eb_per_time_source,&unicast_per_neighbor_link_based,&default_common}"
    else
      # enable the `rpl_storing` rule
      CFLAGS += -DORCHESTRA_CONF_RULES="{&eb_per_time_source,&unicast_per_neighbor_rpl_storing,&default_common}"
    endif
  else
    ifeq ($(MAKE_WITH_LINK_BASED_ORCHESTRA),1)
      $(error "Inconsistent configuration")
    endif
  endif
endif

ifeq ($(MAKE_WITH_SECURITY),1)
CFLAGS += -DWITH_SECURITY=1
endif

include $(CONTIKI)/Makefile.include
