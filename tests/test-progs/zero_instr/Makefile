CC=aarch64-linux-gnu-gcc
CFLAGS=-static -g -I../../../include/

SIZE=4096

.PHONY: all
all: zero_instr zero_loop

zero_instr: test.c zero.o ../../../util/m5/libm5.a
	$(CC) -DSIZE=$(SIZE) -DUSE_$@ $(CFLAGS) $^ -o $@
zero_loop: test.c zero.o ../../../util/m5/libm5.a
	$(CC) -DSIZE=$(SIZE) -DUSE_$@ $(CFLAGS) $^ -o $@

zero.o:
	$(CC) $(CFLAGS) -O3 zero.c -c -o zero.o

test_print: test_print.o zero.o
	$(CC) $(CFLAGS) $^ -o $@

test_large: test_large.o zero.o ../../../util/m5/libm5.a
test_large: newlib_memset.o newlib_no_dc_zva_memset.o
	$(CC) $(CFLAGS) $^ -o $@
