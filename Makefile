
# lorawan_node make project

APP := lorawan_node
FUOTA_TEST_01_APP := fuota-test-01
PERIODIC_UPLINK_LPP_APP := periodic-uplink-lpp

SRCDIR := .\
 src/apps/LoRaMac/common \
 src/apps/LoRaMac/common/LmHandler \
 src/apps/LoRaMac/common/LmHandler/packages \
 src/boards/ \
 src/boards/linux \
 src/boards/mcu \
 src/mac \
 src/mac/region \
 src/peripherals/soft-se \
 src/radio/sx126x \
 src/radio \
 src/system
SRCCFILE := $(addsuffix /*.c,$(SRCDIR))
SRCHINCLUDE := $(addprefix -I,$(SRCDIR))

OBJDIR := build
OBJSUBDIR := $(addprefix build/,$(SRCDIR))
SRC = $(wildcard $(SRCCFILE))
OBJS := $(patsubst %.c,$(OBJDIR)/%.o,$(SRC))
OBJS += build/src/apps/LoRaMac/periodic-uplink-lpp/linux/main.o
# OBJS += build/src/apps/ping-pong/linux/main.o

FUOTA_TEST_01_OBJS += $(OBJS) build/src/apps/LoRaMac/fuota-test-01/linux/main.o
FUOTA_TEST_01_SRCHINCLUDE += $(SRCHINCLUDE) src/apps/LoRaMac/fuota-test-01/
PERIODIC_UPLINK_LPP_OBJS += $(OBJS) build/src/apps/LoRaMac/periodic-uplink-lpp/linux/main.o
PERIODIC_UPLINK_LPP_SRCHINCLUDE += $(SRCHINCLUDE) src/apps/LoRaMac/periodic-uplink-lpp/

CFLAGS += -Wall -g -std=gnu99 #-pipe -mno-branch-likely -mips32r2 -mtune=24kec -mdsp -fno-caller-saves -fhonour-copts -Wno-error=unused-but-set-variable -Wno-error=unused-result -msoft-float #  -std=gnu99
CFLAGS += -DSX1262MBXDAS -DLORAMAC_CLASSB_ENABLED -DSOFT_SE -DSECURE_ELEMENT_PRE_PROVISIONED -DREGION_CN470 #-DREGION_EU868 #-DREGION_CN470
LDFLAGS += -lm -lpthread -lrt

all: $(OBJDIR) $(APP)
$(OBJDIR):
	@mkdir -p $(OBJSUBDIR) $(SRCDIR) build/src/apps/LoRaMac/periodic-uplink-lpp/linux/ build/src/apps/ping-pong/linux/
$(APP): $(OBJDIR)/$(APP)
$(OBJDIR)/$(APP): $(OBJS)
	$(CC) $(OBJS) $(LDFLAGS) -o $@
$(OBJDIR)/%.o: %.c
	$(CC) -c $(CFLAGS) $(SRCHINCLUDE) $< -o $@
clean:
	$(RM) -rf $(OBJDIR)

fuota-test-01: $(OBJDIR) $(FUOTA_TEST_01_APP)
$(OBJDIR):
	@mkdir -p $(OBJSUBDIR) $(SRCDIR) build/src/apps/LoRaMac/fuota-test-01/linux/ build/src/apps/ping-pong/linux/
$(FUOTA_TEST_01_APP): $(OBJDIR)/$(FUOTA_TEST_01_APP)
$(OBJDIR)/$(FUOTA_TEST_01_APP): $(FUOTA_TEST_01_OBJS)
	$(CC) $(FUOTA_TEST_01_OBJS) $(LDFLAGS) -o $@
$(OBJDIR)/%.o: %.c
	$(CC) -c $(CFLAGS) $(FUOTA_TEST_01_SRCHINCLUDE) $< -o $@

periodic-uplink-lpp: $(OBJDIR) $(PERIODIC_UPLINK_LPP_APP)
$(OBJDIR):
	@mkdir -p $(OBJSUBDIR) $(SRCDIR) build/src/apps/LoRaMac/periodic-uplink-lpp/linux/ build/src/apps/ping-pong/linux/
$(PERIODIC_UPLINK_LPP_APP): $(OBJDIR)/$(PERIODIC_UPLINK_LPP_APP)
$(OBJDIR)/$(PERIODIC_UPLINK_LPP_APP): $(PERIODIC_UPLINK_LPP_OBJS)
	$(CC) $(PERIODIC_UPLINK_LPP_OBJS) $(LDFLAGS) -o $@
$(OBJDIR)/%.o: %.c
	$(CC) -c $(CFLAGS) $(PERIODIC_UPLINK_LPP_SRCHINCLUDE) $< -o $@

.PHONY: all clean
