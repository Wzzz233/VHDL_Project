# C+ RK3568 FP16 driver.  Host tests deliberately do not require RKNN.

CC ?= gcc
CFLAGS ?= -Wall -Wextra -Werror -O2 -std=c11

CORE_TEST := test_cplus_engine

all: $(CORE_TEST)

$(CORE_TEST): test_cplus_engine.c cplus_engine.c cplus_core.h
	$(CC) $(CFLAGS) -o $@ test_cplus_engine.c cplus_engine.c -lm

test: $(CORE_TEST)
	./$(CORE_TEST)

clean:
	rm -f $(CORE_TEST) cplus_rk3568_driver

.PHONY: all test clean
