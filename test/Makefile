CC ?= cc
CFLAGS ?= -std=c11 -Wall -Wextra -Werror -O2
CPPFLAGS ?= -I../main

.PHONY: test clean

test: test_line_math
	./test_line_math

test_line_math: test_line_math.c ../main/line_math.c ../main/line_math.h
	$(CC) $(CFLAGS) $(CPPFLAGS) -o $@ test_line_math.c ../main/line_math.c -lm

clean:
	rm -f test_line_math
