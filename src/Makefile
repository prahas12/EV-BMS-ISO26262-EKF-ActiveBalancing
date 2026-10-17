# =============================================================================
# EV-BMS-ISO26262-EKF-ActiveBalancing
# Makefile for BMS Embedded Production Firmware Target
# =============================================================================

CC      ?= gcc
CFLAGS  ?= -std=c99 -Wall -Wextra -Wpedantic -O2 -I.
LDFLAGS ?= -lm

SRCS    := bms_main.c bms_ekf.c bms_fsm.c bms_balancing.c
OBJS    := $(SRCS:.c=.o)
TARGET  := bms_firmware_sim.exe

.PHONY: all clean run

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)

%.o: %.c bms_types.h bms_ekf.h bms_fsm.h bms_balancing.h
	$(CC) $(CFLAGS) -c $< -o $@

run: $(TARGET)
	./$(TARGET)

clean:
	rm -f $(OBJS) $(TARGET)
