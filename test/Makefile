# Host unit tests for CANopenNode, CANopen FD and classic CAN builds.
#
# make        build and run both test programs
# make clean  remove build output

CANOPEN_SRC = ..
CC ?= gcc
CFLAGS = -Wall -Wextra -g -std=gnu11 -I. -I$(CANOPEN_SRC) -DCO_DRIVER_CUSTOM

SOURCES = \
	CO_driver_test.c \
	$(CANOPEN_SRC)/301/CO_ODinterface.c \
	$(CANOPEN_SRC)/301/CO_PDO.c \
	$(CANOPEN_SRC)/301/CO_Emergency.c \
	$(CANOPEN_SRC)/301/CO_USDOserver.c \
	test_fd.c

.PHONY: all run clean

all: run

test_fd: $(SOURCES) *.h $(CANOPEN_SRC)/301/*.h
	$(CC) $(CFLAGS) $(SOURCES) -o $@

test_classic: $(SOURCES) *.h $(CANOPEN_SRC)/301/*.h
	$(CC) $(CFLAGS) -DTEST_CLASSIC $(SOURCES) -o $@

run: test_fd test_classic
	./test_fd
	./test_classic

clean:
	rm -f test_fd test_classic
