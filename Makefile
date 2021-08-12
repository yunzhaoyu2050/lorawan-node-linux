
# lorawan_node make project

APP := lorawan_node

SRCDIR := . common common/LmHandler common/LmHandler/packages mac mac/region soft-se

SRCCFILE := $(addsuffix /*.c,$(SRCDIR))
SRCHINCLUDE := $(addprefix -I,$(SRCDIR))

OBJDIR := build
OBJSUBDIR := $(addprefix build/,$(SRCDIR))
SRC = $(wildcard $(SRCCFILE))
# SRCO = $(notdir $(SRC))
OBJS := $(patsubst %.c,$(OBJDIR)/%.o,$(SRC))

CFLAGS += -Wall -g -std=gnu99 -pipe -mno-branch-likely -mips32r2 -mtune=24kec -mdsp -fno-caller-saves -fhonour-copts -Wno-error=unused-but-set-variable -Wno-error=unused-result -msoft-float
CFLAGS += $(SRCHINCLUDE)
LDFLAGS += -lm -lpthread -lrt

all: $(OBJDIR) $(APP)
$(OBJDIR):
	@mkdir -p $(OBJSUBDIR)
$(APP): $(OBJDIR)/$(APP)
$(OBJDIR)/$(APP): $(OBJS)
	$(CC) $(OBJS) $(LDFLAGS) -o $@
$(OBJDIR)/%.o: %.c
	$(CC) -c $(CFLAGS) $< -o $@
clean:
	$(RM) -rf $(OBJDIR)
.PHONY: all clean
