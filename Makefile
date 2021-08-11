
APP := lorawan_node
OBJDIR := build
SRC=$(wildcard *.c ./common/*.c ./common/LmHandler/*.c ./common/LmHandler/packages/*.c ./mac/*.c ./mac/region/*.c ./soft-se/*.c)
OBJS   := $(patsubst %.c,$(OBJDIR)/%.o,$(SRC))
CFLAGS  += -Wall -g -std=c99
LDFLAGS += -luci -lm -lpthread -lrt
all: $(OBJDIR) $(APP)
$(OBJDIR):
	@mkdir -p $@
$(APP): $(OBJDIR)/$(APP)
$(OBJDIR)/$(APP): $(OBJS)
	$(CC) $(OBJS) $(LDFLAGS) -o $@
$(OBJDIR)/%.o: %.c
	$(CC) -c $(CFLAGS) $< -o $@
clean:
	$(RM) -rf $(OBJDIR)
.PHONY: all clean
