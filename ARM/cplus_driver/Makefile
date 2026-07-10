# C+ RK3568 FP16 driver.  Host tests deliberately do not require RKNN.

CC ?= gcc
CFLAGS ?= -Wall -Wextra -Werror -O2 -std=c11

CORE_TEST := test_cplus_engine
ASYNC_TEST := test_cplus_async_primitives

all: $(CORE_TEST) $(ASYNC_TEST)

$(CORE_TEST): test_cplus_engine.c cplus_engine.c cplus_core.h
	$(CC) $(CFLAGS) -o $@ test_cplus_engine.c cplus_engine.c -lm

$(ASYNC_TEST): test_cplus_async_primitives.c cplus_frame_pool.c cplus_frame_pool.h cplus_latest_queue.c cplus_latest_queue.h
	$(CC) $(CFLAGS) -pthread -o $@ test_cplus_async_primitives.c cplus_frame_pool.c cplus_latest_queue.c

test: $(CORE_TEST) $(ASYNC_TEST)
	./$(CORE_TEST)
	./$(ASYNC_TEST)

clean:
	rm -f $(CORE_TEST) $(ASYNC_TEST) test_cplus_async_tsan cplus-rk3568-driver

.PHONY: all test clean
