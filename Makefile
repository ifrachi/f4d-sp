CONTIKI_PROJECT = sp_v1

TARGET = cc26x0-cc13x0
BOARD = sensortag/cc2650

CORE_SDK=../sdk/coresdk_cc13xx_cc26xx


CFLAGS += -I$(CORE_SDK)/include
LDFLAGS += -L$(CORE_SDK)/lib

MODULES_REL += $(TARGET)

#CFLAGS += -g
CFLAGS += -w

MAKE_MAC = MAKE_MAC_TSCH
MAKE_ROUTING = MAKE_ROUTING_RPL_CLASSIC
MAKE_NET=MAKE_NET_IPV6
MODULES += os/services/orchestra
`

all: $(CONTIKI_PROJECT)

CONTIKI=../../../../contiki-ng



include $(CONTIKI)/Makefile.include
